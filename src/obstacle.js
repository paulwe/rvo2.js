var RVO = RVO || {};

RVO.Obstacle = function() {
  this.isConvex = false;
  this.nextobstacle = 0;
  this.point = [0, 0];
  this.prevObstacle = 0;
  this.unitDir = [0, 0];
  this.id = 0;
}
