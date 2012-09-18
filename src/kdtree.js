var RVO = RVO || {};

RVO.KdTree = function(sim) {
  this.sim = sim;
  this.agents = [];
  this.agentTree = [];
  this.obstacleTree = 0;
}

RVO.KdTree.MAX_LEAF_SIZE = 10;

RVO.KdTree.prototype.buildAgentTree = function() {
  if (this.agents.length < this.sim.agents.length) {
    for (var i = this.agents.length, len = this.sim.agents.length; i < len; ++ i) {
      this.agents.push(this.sim.agents[i]);
    }
  }

  this.agentTree = [];
  if (this.agents.length) {
    this.buildAgentTreeRecursive(0, this.agents.length, 0);
  }
}

RVO.KdTree.prototype.buildAgentTreeRecursive = function(begin, end, node) {
  var agent = this.agentTree[node] = new RVO.KdTree.AgentTreeNode;
  agent.begin = begin;
  agent.end = end;

  agent.minX = agent.maxX = this.agents[begin].position[0];
  agent.minY = agent.maxY = this.agents[begin].position[1];

  for (var i = begin + 1; i < end; ++ i) {
    agent.maxX = Math.max(agent.maxX, this.agents[i].position[0]);
    agent.minX = Math.min(agent.minX, this.agents[i].position[0]);
    agent.maxY = Math.max(agent.maxY, this.agents[i].position[1]);
    agent.minY = Math.min(agent.minY, this.agents[i].position[1]);
  }

  if (end - begin > RVO.KdTree.MAX_LEAF_SIZE) {
    var isVertical = agent.maxX - agent.minX > agent.maxY - agent.minY
      , splitValue = isVertical ? .5 * (agent.maxX + agent.minX) : .5 * (agent.maxY + agent.minY)
      , left = begin
      , right = end - 1;

    while (true) {
      while (left <= right && (isVertical ? this.agents[left].position[0] : this.agents[left].position[1]) < splitValue) {
        ++ left;
      }

      while (right >= left && (isVertical ? this.agents[right].position[0] : this.agents[right].position[1]) >= splitValue) {
        -- right;
      }

      if (left > right) {
        break;
      }
      else {
        var tmp = this.agents[right];
        this.agents[right] = this.agents[left];
        this.agents[left] = tmp;
        ++ left;
        -- right;
      }
    }

    var leftSize = left - begin;

    if (leftSize == 0) {
      ++ leftSize;
      ++ left;
      ++ right;
    }

    agent.left = node + 1;
    agent.right = node + 1 + (2 * leftSize - 1);

    this.buildAgentTreeRecursive(begin, left, agent.left);
    this.buildAgentTreeRecursive(left, end, agent.right);
  }
}

RVO.KdTree.prototype.buildObstacleTree = function() {
  var obstacles = [];

  for (var i = 0, len = this.sim.obstacles.length; i < len; ++ i) {
    obstacles[i] = this.sim.obstacles[i];
  }

  this.obstacleTree = this.buildObstacleTreeRecursive(obstacles);
}

RVO.KdTree.prototype.buildObstacleTreeRecursive = function(obstacles) {
  if (!obstacles.length) {
    return 0;
  }
  else {
    var node = new RVO.KdTree.ObstacleTreeNode
      , optimalSplit = 0
      , obstaclesLength = obstacles.length
      , minLeft = obstaclesLength
      , minRight = obstaclesLength;

    for (var i = 0; i < obstaclesLength; ++ i) {
      var leftSize = 0
        , rightSize = 0
        , obstacleI1 = obstacles[i]
        , obstacleI2 = obstacleI1.nextObstacle;

      for (var j = 0; j < obstaclesLength; ++ j) {
        if (i != j) {
          var obstacleJ1 = obstacles[j]
            , obstacleJ2 = obstacleJ1.nextObstacle
            , j1LeftOfI = RVO.Vector.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point)
            , j2LeftOfI = RVO.Vector.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

          if (j1LeftOfI >= - RVO.EPSILON && j2LeftOfI >= - RVO.EPSILON) {
            ++ leftSize;
          }
          else if (j1LeftOfI <= RVO.EPSILON && j2LeftOfI <= RVO.EPSILON) {
            ++ rightSize;
          }
          else {
            ++ leftSize;
            ++ rightSize;
          }

          if (Math.max(leftSize, rightSize) >= Math.min(minLeft, minRight) || Math.min(leftSize, rightSize) >= Math.min(minLeft, minRight)) {
            break;
          }
        }
      }

      if (Math.max(leftSize, rightSize) < Math.max(minLeft, minRight) || Math.min(leftSize, rightSize) < Math.min(minLeft, minRight)) {
        minLeft = leftSize;
        minRight = rightSize;
        optimalSplit = i;
      }
    }

    var leftObstacles = []
      , rightObstacles = []
      , i = optimalSplit
      , obstacleI1 = obstacles[i]
      , obstacleI2 = obstacleI1.nextObstacle;

    for (var j = 0, len = obstacles.length; j < len; ++ j) {
      if (i != j) {
        var obstacleJ1 = obstacles[j]
          , obstacleJ2 = obstacleJ1.nextObstacle
          , j1LeftOfI = RVO.Vector.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point)
          , j2LeftOfI = RVO.Vector.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

        if (j1LeftOfI >= - RVO.EPSILON && j2LeftOfI >= - RVO.EPSILON) {
          leftObstacles.push(obstacles[j]);
        }
        else if (j1LeftOfI <= RVO.EPSILON && j2LeftOfI <= RVO.EPSILON) {
          rightObstacles.push(obstacles[j]);
        }
        else {
          var t = RVO.Vector.det(RVO.Vector.subtract(obstacleI2.point, obstacleI1.point), RVO.Vector.subtract(obstacleJ1.point, obstacleI1.point)) / RVO.Vector.det(RVO.Vector.subtract(obstacleI2.point, obstacleI1.point), RVO.Vector.subtract(obstacleJ1.point, obstacleJ2.point))
            , splitPoint = RVO.Vector.add(RVO.Vector.multiply(RVO.Vector.subtract(obstacleJ1.point, obstacleJ2.point), t), obstacleJ1.point)
            , newObstacle = new RVO.Obstacle();

          newObstacle.point = splitPoint;
          newObstacle.prevObstacle = obstacleJ1;
          newObstacle.nextObstacle = obstacleJ2;
          newObstacle.isConvex = true;
          newObstacle.unitTDir = obstacleJ1.unitDir;
          newObstacle.id = this.sim.obstacles.length;

          this.sim.obstacles.push(newObstacle);

          obstacleJ1.nextObstacle = newObstacle;
          obstacleJ2.prevObstacle = newObstacle;

          if (j1LeftOfI > 0) {
            leftObstacles.push(obstacleJ1);
            rightObstacles.push(newObstacle);
          }
          else {
            rightObstacles.push(obstacleJ1);
            leftObstacles.push(newObstacle);
          }
        }
      }
    }

    node.obstacle = obstacleI1;
    node.left = this.buildObstacleTreeRecursive(leftObstacles);
    node.right = this.buildObstacleTreeRecursive(rightObstacles);
    return node;
  }
}

RVO.KdTree.prototype.computeAgentNeighbors = function(agent, rangeSq) {
  this.queryAgentTreeRecursive(agent, rangeSq, 0);
}

RVO.KdTree.prototype.computeObstacleNeighbors = function(agent, rangeSq) {
  this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree);
}

RVO.KdTree.prototype.queryAgentTreeRecursive = function(agent, rangeSq, node) {
  var nodeAgent = this.agentTree[node]
    , newRangeSq = rangeSq;
  if (nodeAgent.end - nodeAgent.begin <= RVO.KdTree.MAX_LEAF_SIZE) {
    for (var i = nodeAgent.begin, len = nodeAgent.end; i < len; ++ i) {
      newRangeSq = agent.insertAgentNeighbor(this.agents[i], newRangeSq);
    }
  }
  else {
    var distSqLeft = 0
      , distSqRight = 0
      , leftAgent = this.agentTree[nodeAgent.left]
      , rightAgent = this.agentTree[nodeAgent.right];

    if (agent.position[0] < leftAgent.minX) {
      distSqLeft += RVO.sqr(leftAgent.minX - agent.position[0]);
    }
    else if (agent.position[0] > leftAgent.maxX) {
      distSqLeft += RVO.sqr(agent.position[0] - leftAgent.maxX);
    }

    if (agent.position[1] < leftAgent.minY) {
      distSqLeft += RVO.sqr(leftAgent.minY - agent.position[1]);
    }
    else if (agent.position[1] > leftAgent.maxY) {
      distSqLeft += RVO.sqr(agent.position[1] - leftAgent.maxY);
    }

    if (agent.position[0] < rightAgent.minX) {
      distSqRight += RVO.sqr(rightAgent.minX - agent.position[0]);
    }
    else if (agent.position[0] > rightAgent.maxX) {
      distSqRight += RVO.sqr(agent.position[0] - rightAgent.maxX);
    }

    if (agent.position[1] < rightAgent.minY) {
      distSqRight += RVO.sqr(rightAgent.minY - agent.position[1]);
    }
    else if (agent.position[1] > rightAgent.maxY) {
      distSqRight += RVO.sqr(agent.position[1] - rightAgent.maxY);
    }

    if (distSqLeft < distSqRight) {
      if (distSqLeft < rangeSq) {
        newRangeSq = this.queryAgentTreeRecursive(agent, newRangeSq, nodeAgent.left);

        if (distSqRight < rangeSq) {
          newRangeSq = this.queryAgentTreeRecursive(agent, newRangeSq, nodeAgent.right);
        }
      }
    }
    else {
      if (distSqRight < rangeSq) {
        newRangeSq = this.queryAgentTreeRecursive(agent, newRangeSq, nodeAgent.right);

        if (distSqLeft < rangeSq) {
          newRangeSq = this.queryAgentTreeRecursive(agent, newRangeSq, nodeAgent.left);
        }
      }
    }
  }
  return newRangeSq;
}

RVO.KdTree.prototype.queryObstacleTreeRecursive = function(agent, rangeSq, node) {
  if (node == 0) {
    return;
  }
  else {
    var obstacle1 = node.obstacle
      , obstacle2 = obstacle1.nextObstacle
      , agentLeftOfLine = RVO.Vector.leftOf(obstacle1.point, obstacle2.point, agent.position)
      , distSqLine = RVO.sqr(agentLeftOfLine) / RVO.Vector.absSq(RVO.Vector.subtract(obstacle2.point, obstacle1.point));

    this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0 ? node.left : node.right);

    if (distSqLine < rangeSq) {
      if (agentLeftOfLine < 0) {
        agent.insertObstacleNeighbor(node.obstacle, rangeSq);
      }

      this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0 ? node.right : node.left);
    }
  }
}

RVO.KdTree.prototype.queryVisibility = function(q1, q2, radius) {
  return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree)
}

RVO.KdTree.prototype.queryVisibilityRecursive = function(q1, q2, radius, node) {
  if (node.obstacleNo == -1) {
    return true;
  }
  else {
    var obstacle = this.sim.obstacles[node.obstacleNo]
      , q1LeftOfI = RVO.Vector.leftOf(obstacle.point1, obstacle.point2, q1)
      , q2LeftOfI = RVO.Vector.leftOf(obstacle.point1, obstacle.point2, q2);

    if (q1LeftOfI >= 0 && q2LeftOfI >= 0) {
      return this.queryVisibilityRecursive(q1, q2, radius, node.left);
    }
    else if (q1LeftOfI <= 0 && q2LeftOfI <= 0) {
      return this.queryVisibilityRecursive(q1, q2, radius, node.right);
    }
    else {
      var point1LeftOfQ = RVO.Vector.leftOf(q1, q2, obstacle.point1)
        , point2LeftOfQ = RVO.Vector.leftOf(q1, q2, obstacle.point2)
        , invLengthQ = 1 / RVO.Vector.absSq(RVO.Vector.subtract(q2, q1));;

      return point1LeftOfQ * point2LeftOfQ >= 0 && RVO.sqr(point1LeftOfQ) * invLengthQ >= RVO.sqr(radius) && RVO.sqr(point2LeftOfQ) * invLengthQ >= RVO.sqr(radius) && this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right);
    }
  }
}

RVO.KdTree.AgentTreeNode = function() {
  this.begin = 0;
  this.end = 0;
  this.minX = 0;
  this.minY = 0;
  this.maxX = 0;
  this.maxY = 0;
  this.left = 0;
  this.right = 0;
}

RVO.KdTree.ObstacleTreeNode = function() {
  this.left = null;
  this.right = null;
  this.obstacle = 0;
}