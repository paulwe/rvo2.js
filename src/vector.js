var RVO = RVO || {};

RVO.Vector = {};

RVO.Vector.invert = function(a) {
  return [- a[0], - a[1]];
}

RVO.Vector.multiplyVector = function(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

RVO.Vector.multiply = function(a, b) {
  return [a[0] * b, a[1] * b];
}

RVO.Vector.divide = function(a, b) {
  return [a[0] / b, a[1] / b];
}

RVO.Vector.add = function(a, b) {
  return [a[0] + b[0], a[1] + b[1]];
}

RVO.Vector.subtract = function(a, b) {
  return [a[0] - b[0], a[1] - b[1]];
}

RVO.Vector.shift = function(a, b) {
  a[0] += b[0];
  a[1] += b[1];
  return a;
}

RVO.Vector.set = function(a, b) {
  a[0] = b[0];
  a[1] = b[1];
  return a;
}

RVO.Vector.abs = function(a) {
  return Math.sqrt(RVO.Vector.multiplyVector(a, a));
}

RVO.Vector.absSq = function(a) {
  return RVO.Vector.multiplyVector(a, a);
}

RVO.Vector.det = function(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

RVO.Vector.normalize = function(a) {
  return RVO.Vector.divide(a, RVO.Vector.abs(a));
}

RVO.Vector.leftOf = function(a, b, c) {
  return RVO.Vector.det(RVO.Vector.subtract(a, c), RVO.Vector.subtract(b, a));
}

RVO.Vector.distSqPointLineSegment = function(a, b, c) {
  var ba = RVO.Vector.subtract(b, a)
    , r = RVO.Vector.multiplyVector(RVO.Vector.subtract(c, a), ba) / RVO.Vector.absSq(ba);

  if (r < 0) {
    return RVO.Vector.absSq(RVO.Vector.subtract(c, a));
  }
  else if (r > 1) {
    return RVO.Vector.absSq(RVO.Vector.subtract(c, b));
  }
  else {
    return RVO.Vector.absSq(RVO.Vector.subtract(c, RVO.Vector.add(RVO.Vector.multiply(ba, r), a)));
  }
}