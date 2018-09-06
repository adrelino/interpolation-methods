var hh = 300;
var ww = 300;
var pad = 10;
var composite=true;
var points = [{x: pad, y: hh-pad}, {x: pad, y: pad}, {x: ww-pad, y: pad}, {x: ww-pad, y: hh-pad}];

if(composite){
  points = [{x: pad, y: hh/2}, {x: pad, y: pad}, {x: ww/2, y: pad}, {x: ww/2, y: hh/2}, 
            {x: ww/2, y: hh-pad}, {x: ww-pad, y: hh-pad}, {x: ww-pad, y: hh/2}];
}

points[2].x = 0.5*(points[2].x + points[1].x);

var arr = [];
arr.push(new BSpline("#vis0",JSON.parse(JSON.stringify(points)),true,composite,0));
arr.push(new BSpline("#vis1",JSON.parse(JSON.stringify(points)),false,composite,2));

var continuity = d3.select("#continuity");
continuity.on('change',function(e){
  arr[0].setContinuity(this.value);
});

var t = 0.75;
function updateAll(t){
  for(var i=0; i< arr.length; i++){
        arr[i].update(t);
  }
}
updateAll(t);