var RVO = RVO || {};

RVO.EPSILON = .00001;

RVO.defaultAgent = {
  neighborDist: 50,
  maxNeighbors: 5,
  maxSpeed: 1,
  radius: 10,
  timeHorizon: 10,
  velocity: [0, 0]
}

RVO.sqr = function(a) {
  return a * a;
}

if (typeof define === "function" && define.amd && define.amd.RVO) {
  define('RVO', [], function () {
    return RVO;
  });
}

if (typeof module === 'object' && module.exports) {
  module.exports = RVO;
}
