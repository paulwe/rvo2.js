#!/usr/bin/env node

var compressor = require('node-minify')
  , fs = require('fs');

var srcFiles = [
  'src/agent.js',
  'src/kdtree.js',
  'src/obstacle.js',
  'src/rvo.js',
  'src/simulator.js',
  'src/vector.js'
];

var license = "/*\n" + fs.readFileSync('LICENSE').toString() + "\n*/\n\n";

new compressor.minify({
  type: 'yui-js',
  fileIn: srcFiles,
  fileOut: '/tmp/rvo2.min.js',
  callback: function(err){
    fs.writeFileSync('lib/rvo2.min.js', license.concat(fs.readFileSync('/tmp/rvo2.min.js').toString()));
    console.log(err || 'Done buliding rvo2.min.js');
  }
});

new compressor.minify({
  type: 'no-compress',
  fileIn: srcFiles,
  fileOut: '/tmp/rvo2.js',
  callback: function(err){
    fs.writeFileSync('lib/rvo2.js', license.concat(fs.readFileSync('/tmp/rvo2.js').toString()));
    console.log(err || 'Done building rvo2.js');
  }
});