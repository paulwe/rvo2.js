var RVO = RVO || {};

RVO.Simulator = function(timeStep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity) {
  this.timeStep = timeStep;
  this.agents = [];
  this.globalTime = 0;
  this.obstacles = [];
  this.kdTree = new RVO.KdTree(this);
  this.agentDefaults = {};

  this.agentDefaults.neighborDist = neighborDist || RVO.agentDefaults.neighborDist;
  this.agentDefaults.maxNeighbors = maxNeighbors || RVO.agentDefaults.maxNeighbors;
  this.agentDefaults.maxSpeed = maxSpeed || RVO.agentDefaults.maxSpeed;
  this.agentDefaults.radius = radius || RVO.agentDefaults.radius;
  this.agentDefaults.timeHorizon = timeHorizon || RVO.agentdefaults.timeHorizon;
  this.agentDefaults.timeHorizonObst = timeHorizonObst || RVO.agentDefaults.timeHorizonObst;
  this.agentDefaults.velocity = velocity || RVO.agentdefaults.velocity;
}

RVO.Simulator.prototype.addAgent = function(position, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity) {
  var agent = new RVO.Agent(this);

  agent.position = position || this.agentDefaults.position;
  agent.maxNeighbors = maxNeighbors || this.agentDefaults.maxNeighbors;
  agent.maxSpeed = maxSpeed || this.agentDefaults.maxSpeed;
  agent.neighborDist = neighborDist || this.agentDefaults.neighborDist;
  agent.radius = radius || this.agentDefaults.radius;
  agent.timeHorizon = timeHorizon || this.agentDefaults.timeHorizon;
  agent.timeHorizonObst = timeHorizonObst || this.agentDefaults.timeHorizonObst;
  agent.velocity = velocity || this.agentDefaults.velocity;
  agent.id = this.agents.length;

  this.agents.push(agent);

  return this.agents.length - 1;
}

RVO.Simulator.prototype.addObstacle = function(vertices) {
  if (vertices.length < 2) {
    throw new Error('Obstacle created with less than two vertices');
  }

  var obstacleNo = this.obstacles.length;

  for (var i = 0, len = vertices.length; i < len; ++ i) {
    var obstacle = new RVO.Obstacle();
    obstacle.point = vertices[i];
    if (i != 0) {
      obstacle.prevObstacle = this.obstacles[this.obstacles.length - 1];
      obstacle.prevObstacle.nextObstacle = obstacle;
    }
    if (i == vertices.length - 1) {
      obstacle.nextObstacle = this.obstacles[obstacleNo];
      obstacle.nextObstacle.prevObstacle = obstacle;
    }
    obstacle.unitDir = RVO.Vector.normalize(RVO.Vector.subtract(vertices[i == vertices.length - 1 ? 0 : i + 1], vertices[i]));

    if (vertices.length == 2) {
      obstacle.isConvex = true;
    }
    else {
      obstacle.isConvex = (RVO.Vector.leftOf(vertices[i == 0 ? vertices.length - 1 : i - 1], vertices[i], vertices[i == vertices.length - 1 ? 0 : i + 1]) >= 0);
    }
    obstacle.id = this.obstacles.length;
    this.obstacles.push(obstacle);
  }

  return obstacleNo;
}

RVO.Simulator.prototype.processObstacles = function() {
  this.kdTree.buildObstacleTree();
}

RVO.Simulator.prototype.doStep = function() {
  this.kdTree.buildAgentTree();

  for (var i = 0, len = this.agents.length; i < len; ++ i) {
    this.agents[i].computeNeighbors();
    this.agents[i].computeNewVelocity();
  }

  for (var i = 0, len = this.agents.length; i < len; ++ i) {
    this.agents[i].update();
  }

  this.globalTime += this.timeStep;
}
