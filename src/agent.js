var RVO = RVO || {};

RVO.Agent = function(sim) {
  this.sim = sim;

  this.agentNeighbors = [];
  this.maxNeighbors = 0;
  this.maxSpeed = 0;
  this.neighborDist = 0;
  this.newVelocity = [0, 0];
  this.obstacleNeighbors = [];
  this.orcaLines = [];
  this.position = [0, 0];
  this.sim = sim;
  this.timeHorizon = 0;
  this.timeHorizonObst = 0;
  this.velocity = [0, 0];
  this.id = 0;
}

RVO.Agent.prototype.computeNeighbors = function() {
  this.obstacleNeighbors = [];
  var rangeSq = RVO.sqr(this.timeHorizonObst * this.maxSpeed + this.radius);
  this.sim.kdTree.computeObstacleNeighbors(this, rangeSq);

  this.agentNeighbors = [];
  if (this.maxNeighbors > 0) {
    rangeSq = RVO.sqr(this.neighborDist);
    this.sim.kdTree.computeAgentNeighbors(this, rangeSq);
  }
}

RVO.Agent.prototype.computeNewVelocity = function() {
  this.orcaLines = [];
  var invTimeHorizonObst = 1 / this.timeHorizonObst;

  for (var i = 0, ilen = this.obstacleNeighbors.length; i < ilen; ++ i) {
    var obstacle1 = this.obstacleNeighbors[i][1]
      , obstacle2 = obstacle1.nextObstacle
      , relativePosition1 = RVO.Vector.subtract(obstacle1.point, this.position)
      , relativePosition2 = RVO.Vector.subtract(obstacle2.point, this.position)
      , alreadyCovered = false;

    for (var j = 0, jlen = this.orcaLines.length; j < jlen; ++ j) {
      if (RVO.Vector.det(RVO.Vector.multiply(RVO.Vector.subtract(relativePosition1, this.orcaLines[j][0]), invTimeHorizonObst), this.orcaLines[j][1]) - invTimeHorizonObst * this.radius >= - RVO.EPSILON && RVO.Vector.det(RVO.Vector.multiply(RVO.Vector.subtract(relativePosition2, this.orcaLines[j][0]), invTimeHorizonObst), this.orcaLines[j][1]) - invTimeHorizonObst * this.radius >= - RVO.EPSILON) {
        alreadyCovered = true;
        break;
      }
    }

    if (alreadyCovered) {
      continue;
    }

    var distSq1 = RVO.Vector.absSq(relativePosition1)
      , distSq2 = RVO.Vector.absSq(relativePosition2)
      , radiusSq = RVO.sqr(this.radius)
      , obstacleVector = RVO.Vector.subtract(obstacle2.point, obstacle1.point)
      , s = RVO.Vector.dotProduct(RVO.Vector.invert(relativePosition1), obstacleVector) / RVO.Vector.absSq(obstacleVector)
      , distSqLine = RVO.Vector.absSq(RVO.Vector.subtract(RVO.Vector.invert(relativePosition1), RVO.Vector.multiply(obstacleVector, s)))
      , line = new Array(2);

    if (s < 0 && distSq1  <= radiusSq) {
      if (obstacle1.isConvex) {
        line[0] = [0, 0];
        line[1] = RVO.Vector.normalize([- relativePosition1[1], relativePosition1[0]]);
        this.orcaLines.push(line);
      }
      continue;
    }
    else if (s > 1 && distSq2 <= radiusSq) {
      if (obstacle2.isConvex && RVO.Vector.det(relativePosition2, obstacle2.unitDir) >= 0) {
        line[0] = [0, 0];
        line[1] = RVO.Vector.normalize([- relativePosition2[1], relativePosition2[0]]);;
        this.orcaLines.push(line);
      }
      continue;
    }
    else if (s >= 0 && s < 1 && distSqLine <= radiusSq) {
      line[0] = [0, 0];
      line[1] = RVO.Vector.invert(obstacle1.unitDir);
      this.orcaLines.push(line);
      continue;
    }

    var leftLegDirection, rightLegDirection;

    if (s < 0 && distSqLine <= radiusSq) {
      if (!obstacle1.isConvex) {
        continue;
      }

      obstacle2 = obstacle1;

      var leg1 = Math.sqrt(distSq1 - radiusSq);
      leftLegDirection = RVO.Vector.divide([relativePosition1[0] * leg1 - relativePosition1[1] * this.radius, relativePosition1[0] * this.radius + relativePosition1[1] * leg1], distSq1);
      rightLegDirection = RVO.Vector.divide([relativePosition1[0] * leg1 - relativePosition1[1] * this.radius, - relativePosition1[0] * this.radius + relativePosition1[1] * leg1], distSq1);
    }
    else if (s > 1 && distSqLine <= radiusSq) {
      if (!obstacle2.isConvex) {
        continue;
      }

      obstacle1 = obstacle2;

      var leg2 = Math.sqrt(distSq2 - radiusSq);
      leftLegDirection = RVO.Vector.divide([relativePosition2[0] * leg2 - relativePosition2[1] * this.radius, relativePosition2[0] * this.radius + relativePosition2[1] * leg2], distSq2);
      rightLegDirection = RVO.Vector.divide([relativePosition2[0] * leg2 - relativePosition2[1] * this.radius, - relativePosition2[0] * this.radius + relativePosition2[1] * leg2], distSq2);
    }
    else {
      if (obstacle1.isConvex) {
        leg1 = Math.sqrt(distSq1 - radiusSq);
        leftLegDirection = RVO.Vector.divide([relativePosition1[0] * leg1 - relativePosition1[1] * this.radius, relativePosition1[0] * this.radius + relativePosition1[1] * leg1], distSq1);
      }
      else {
        leftLegDirection = RVO.Vector.invert(obstacle1.unitDir);
      }

      if (obstacle2.isConvex) {
        leg2 = Math.sqrt(distSq2 - radiusSq);
        rightLegDirection = RVO.Vector.divide([relativePosition2[0] * leg2 - relativePosition2[1] * this.radius, - relativePosition2[0] * this.radius + relativePosition2[1] * leg2], distSq2);
      }
      else {
        rightLegDirection = obstacle1.unitDir;
      }
    }

    var leftNeighbor = obstacle1.prevObstacle
      , isLeftLegForeign = false
      , isRightLegForeign = false;

    if (obstacle1.isConvex && RVO.Vector.det(leftLegDirection, RVO.Vector.invert(leftNeighbor.unitDir)) >= 0) {
      leftLegDirection = RVO.Vector.invert(leftNeighbor.unitDir);
      isLeftLegForeign = true;
    }

    if (obstacle2.isConvex && RVO.Vector.det(rightLegDirection, obstacle2.unitDir) <= 0) {
      rightLegDirection = RVO.Vector.invert(obstacle2.unitDir);
      isRightLegForeign = true;
    }

    var leftCutoff = RVO.Vector.multiply(RVO.Vector.subtract(obstacle1.point, this.position), invTimeHorizonObst)
      , rightCutoff = RVO.Vector.multiply(RVO.Vector.subtract(obstacle2.point, this.position), invTimeHorizonObst)
      , cutoffVec = RVO.Vector.subtract(rightCutoff, leftCutoff)
      , t = (obstacle1 == obstacle2) ? .5 : RVO.Vector.dotProduct(RVO.Vector.subtract(this.velocity, leftCutoff), cutoffVec) / RVO.Vector.absSq(cutoffVec)
      , tLeft = RVO.Vector.dotProduct(RVO.Vector.subtract(this.velocity, leftCutoff), leftLegDirection)
      , tRight = RVO.Vector.dotProduct(RVO.Vector.subtract(this.velocity, rightCutoff), rightLegDirection);

    if ((t < 0 && tLeft < 0) || (obstacle1 == obstacle2 && tLeft < 0 && tRight < 0)) {
      var unitW = RVO.Vector.normalize(RVO.Vector.subtract(this.velocity, leftCutoff));
      line[1] = [unitW[1], - unitW[0]];
      line[0] = RVO.Vector.add(RVO.Vector.multiply(unitW, this.radius * invTimeHorizonObst), leftCutoff);
      this.orcaLines.push(line);
      continue;
    }
    else if (t > 1 && tRight < 0) {
      var unitW = RVO.Vector.normalize(RVO.Vector.subtract(this.velocity, rightCutoff));
      line[1] = [unitW[1], - unitW[0]];
      line[0] = RVO.Vector.add(RVO.Vector.multiply(unitW, this.radius * invTimeHorizonObst), rightCutoff);
      this.orcaLines.push(line);
      continue;
    }

    var distSqCutoff = (t < 0 || t > 1 || obstacle1 == obstacle2) ? Infinity : RVO.Vector.absSq(RVO.Vector.subtract(this.velocity, RVO.Vector.add(leftCutoff, RVO.Vector.multiply(cutoffVec, t))))
      , distSqLeft = tLeft < 0 ? Infinity : RVO.Vector.absSq(RVO.Vector.subtract(this.velocity, RVO.Vector.add(leftCutoff, RVO.Vector.multiply(leftLegDirection, tLeft))))
      , distSqRight = tRight < 0 ? Infinity : RVO.Vector.absSq(RVO.Vector.subtract(this.velocity, RVO.Vector.add(rightCutoff, RVO.Vector.multiply(rightLegDirection, tRight))));

    if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
      line[1] = RVO.Vector.invert(obstacle1.unitDir);
      line[0] = RVO.Vector.add(leftCutoff, RVO.Vector.multiply([- line[1][1], line[1][0]], this.radius * invTimeHorizonObst));
      this.orcaLines.push(line);
      continue;
    }
    else if (distSqLeft <= distSqRight) {
      if (isLeftLegForeign) {
        continue;
      }

      line[1] = leftLegDirection;
      line[0] = RVO.Vector.add(leftCutoff, RVO.Vector.multiply([- line[1][1], line[1][0]], this.radiuis * invTimeHorizonObst));
      this.orcaLines.push(line);
      continue;
    }
    else {
      if (isRightLegForeign) {
        continue;
      }

      line[1] = RVO.Vector.invert(rightLegDirection);
      line[0] = RVO.Vector.add(rightCutoff, RVO.Vector.multiply([line[1][1], line[1][0]], this.radius * invTimeHorizonObst));
      this.orcaLines.push(line);
      continue;
    }
  }

  var numObstLines = this.orcaLines.length
    , invTimeHorizon = 1 / this.timeHorizon;

  for (var i = 0, len = this.agentNeighbors.length; i < len; ++ i) {
    var other = this.agentNeighbors[i][1]
      , relativePosition = RVO.Vector.subtract(other.position, this.position)
      , relativeVelocity = RVO.Vector.subtract(this.velocity, other.velocity)
      , distSq = RVO.Vector.absSq(relativePosition)
      , combinedRadius = this.radius + other.radius
      , combinedRadiusSq = RVO.sqr(combinedRadius)
      , line = new Array(2);

    if (distSq > combinedRadiusSq) {
      var w = RVO.Vector.subtract(relativeVelocity, RVO.Vector.multiply(relativePosition, invTimeHorizon))
        , wLengthSq = RVO.Vector.absSq(w)
        , dotProduct1 = RVO.Vector.dotProduct(w, relativePosition);

      if (dotProduct1 < 0 && RVO.sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
        var wLength = Math.sqrt(wLengthSq)
          , unitW = RVO.Vector.divide(w, wLength)
          , u = RVO.Vector.multiply(unitW, combinedRadius * invTimeHorizon - wLength);

        line[1] = [unitW[1], - unitW[0]];
      }
      else {
        var leg = Math.sqrt(distSq - combinedRadius);

        if (RVO.Vector.det(relativePosition, w) > 0) {
          line[1] = RVO.Vector.divide([relativePosition[0] * leg - relativePosition[1] * combinedRadius, relativePosition[0] * combinedRadius + relativePosition[1] * leg], distSq)
        }
        else {
          line[1] = RVO.Vector.divide(RVO.Vector.invert([relativePosition[0] * leg - relativePosition[1] * combinedRadius, relativePosition[0] * combinedRadius + relativePosition[1] * leg]), distSq)
        }

        var dotProduct2 = RVO.Vector.dotProduct(relativeVelocity, line[1])
          , u = RVO.Vector.multiply(RVO.Vector.subtract(line[1], relativeVelocity), dotProduct2);
      }
    }
    else {
      var invTimeStep = 1 / this.sim.timeStep
        , w = RVO.Vector.subtract(relativeVelocity, RVO.Vector.multiply(relativePosition, invTimeStep))
        , wLength = RVO.Vector.abs(w)
        , unitW = RVO.Vector.divide(w, wLength)
        , u = RVO.Vector.multiply(unitW, combinedRadius *  invTimeStep - wLength);

      line[1] = [unitW[1], - unitW[0]];
    }

    line[0] = RVO.Vector.add(this.velocity, RVO.Vector.multiply(u, .5));
    this.orcaLines.push(line);
  }

  var lineFail = RVO.Agent.linearProgram2(this.orcaLines, this.maxSpeed, this.prefVelocity, false, this.newVelocity);

  if (lineFail < this.orcaLines.length) {
    RVO.Agent.linearProgram3(this.orcaLines, numObstLines, lineFail, this.maxSpeed, this.newVelocity);
  }
}

RVO.Agent.prototype.insertAgentNeighbor = function(agent, rangeSq) {
  if (this != agent) {
    var distSq = RVO.Vector.absSq(RVO.Vector.subtract(this.position, agent.position));

    if (distSq < rangeSq) {
      if (this.agentNeighbors.length < this.maxNeighbors) {
        this.agentNeighbors.push([distSq, agent]);
      }

      var i = this.agentNeighbors.length - 1;
      while (i != 0 && distSq < this.agentNeighbors[i - 1][0]) {
        this.agentNeighbors[i] = this.agentNeighbors[i - 1];
        -- i;
      }
      this.agentNeighbors[i] = [distSq, agent];

      if (this.agentNeighbors.length == this.maxNeighbors) {
        var rangeSq = this.agentNeighbors[this.agentNeighbors.length - 1][0];
      }
    }
  }

  return rangeSq;
}

RVO.Agent.prototype.insertObstacleNeighbor = function(obstacle, rangeSq) {
  var nextObstacle = obstacle.nextObstacle
    , distSq = RVO.Vector.distSqPointLineSegment(obstacle.point, nextObstacle.point, this.position);

  if (distSq < rangeSq) {
    this.obstacleNeighbors.push([distSq, obstacle]);

    var i = this.obstacleNeighbors.length - 1;
    while (i != 0 && distSq < this.obstacleNeighbors[i - 1][0]) {
      this.obstacleNeighbors[i] = this.obstacleNeighbors[i - 1];
      -- i;
    }
    this.obstacleNeighbors[i] = [distSq, obstacle];
  }
}

RVO.Agent.prototype.update = function() {
  this.velocity = this.newVelocity;
  RVO.Vector.shift(this.position, RVO.Vector.multiply(this.velocity, this.sim.timeStep));
}

RVO.Agent.linearProgram1 = function(lines, lineNo, radius, optVelocity, directionOpt, result) {
  var dotProduct = RVO.Vector.dotProduct(lines[lineNo][0], lines[lineNo][1])
    , discriminant = RVO.sqr(dotProduct) + RVO.sqr(radius) - RVO.Vector.absSq(lines[lineNo][0]);

  if (discriminant < 0) {
    return false;
  }

  var sqrtDiscriminant = Math.sqrt(discriminant)
    , tLeft = - dotProduct - sqrtDiscriminant
    , tRight = - dotProduct + sqrtDiscriminant;

  for (var i = 0; i < lineNo; ++ i) {
    var denominator = RVO.Vector.det(lines[lineNo][1], lines[i][1])
      , numerator = RVO.Vector.det(lines[i][1], RVO.Vector.subtract(lines[lineNo][0], lines[i][0]));

    if (Math.abs(denominator) <= RVO.EPSILON) {
      if (numerator < 0) {
        return false;
      }
      else {
        continue;
      }
    }

    var t = numerator / denominator;

    if (denominator > 0) {
      tRight = Math.min(tRight, t);
    }
    else {
      tLeft = Math.max(tLeft, t);
    }

    if (tLeft > tRight) {
      return false;
    }
  }

  if (directionOpt) {
    if (RVO.Vector.dotProduct(optVelocity, lines[lineNo][1]) > 0) {
      RVO.Vector.set(result, RVO.Vector.add(lines[lineNo][0], RVO.Vector.multiply(lines[lineNo][1], tRight)));
    }
    else {
      RVO.Vector.set(result, RVO.Vector.add(lines[lineNo][0], RVO.Vector.multiply(lines[lineNo][1], tLeft)));
    }
  }
  else {
    t = RVO.Vector.dotProduct(lines[lineNo][1], RVO.Vector.subtract(optVelocity, lines[lineNo][0]));

    if (t < tLeft) {
      RVO.Vector.set(result, RVO.Vector.add(lines[lineNo][0], RVO.Vector.multiply(lines[lineNo][1], tLeft)));
    }
    else if (t > tRight) {
      RVO.Vector.set(result, RVO.Vector.add(lines[lineNo][0], RVO.Vector.multiply(lines[lineNo][1], tRight)));
    }
    else {
      RVO.Vector.set(result, RVO.Vector.add(lines[lineNo][0], RVO.Vector.multiply(lines[lineNo][1], t)));
    }
  }

  return true;
}

RVO.Agent.linearProgram2 = function(lines, radius, optVelocity, directionOpt, result) {
  if (directionOpt) {
    RVO.Vector.set(result, RVO.Vector.multiply(optVelocity, radius));
  }
  else if (RVO.Vector.absSq(optVelocity) > RVO.sqr(radius)) {
    RVO.Vector.set(result, RVO.Vector.multiply(RVO.Vector.normalize(optVelocity), radius));
  }
  else {
    RVO.Vector.set(result, optVelocity);
  }

  for (var i = 0, len = lines.length; i < len; ++ i) {
    if (RVO.Vector.det(lines[i][1], RVO.Vector.subtract(lines[i][0], result)) > 0) {
      var tempResult = result.slice();
      if (!RVO.Agent.linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
        RVO.Vector.set(result, tempResult);
        return i;
      }
    }
  }

  return lines.length;
}

RVO.Agent.linearProgram3 = function(lines, numObstLines, beginLine, radius, result) {
  var distance = 0;

  for (var i = beginLine, len = lines.length; i < len; ++ i) {
    if (RVO.Vector.det(lines[i][1], RVO.Vector.subtract(lines[i][0], result)) > distance) {
      var projLines = lines.slice(0, numObstLines);

      for (var j = numObstLines; j < i; ++ j) {
        var line = new Array(2)
          , determinant = RVO.Vector.det(lines[i][1], lines[j][1]);

        if (Math.abs(determinant) <= RVO.EPSILON) {
          if (RVO.Vector.dotProduct(lines[i][1], lines[j][1]) > 0) {
            continue;
          }
          else {
            line[0] = RVO.Vector.multiply(RVO.Vector.add(lines[i][0], lines[j][0]), .5);
          }
        }
        else {
          line[0] = RVO.Vector.add(lines[i][0], RVO.Vector.multiply(lines[i][1], RVO.Vector.det(lines[j][1], RVO.Vector.subtract(lines[i][0], lines[j][0])) / determinant))
        }

        line[1] = RVO.Vector.normalize(RVO.Vector.subtract(lines[j][1], lines[i][1]));
        projLines.push(line);
      }

      var tempResult = result.slice();
      if (RVO.Agent.linearProgram2(projLines, radius, [- lines[i][1][1], lines[i][1][0]], true, result) < projLines.length) {
        RVO.Vector.set(result, tempResult);
      }

      distance = RVO.Vector.det(lines[i][1], RVO.Vector.subtract(lines[i][0], result));
    }
  }
}