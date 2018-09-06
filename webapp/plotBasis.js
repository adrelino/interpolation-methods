function plotBasis(div,knots,bsplineBasis,k,segments,doBezierSpline){

//  var ww=220;
//  var hh=220;
var pts = knots.length;
var domain = [0,2];//k ? [k-1,pts-k] : [0,1];
var k = k || pts-1;

var timeVar = "u";
if(segments){
  domain = [segments[0][0], segments[segments.length-1][1]];
  timeVar = "&#964"; //&tau
}

var svg = d3.select(div).append("svg")
    .attr("width",ww)
    .attr("height",hh);

//graph basis functions
var margin = {top: 20, right: 0, bottom: 20, left: 20},
    width = ww - margin.left - margin.right,
    height = hh - margin.top - margin.bottom;

var line = d3.svg.line()
	    .x(function(d) {return d.x;})
	    .y(function(d) {return d.y;})
	    .interpolate("linear");

var x = d3.scale.linear()
	.domain(domain)
	.range([margin.left, width]);

// var xinv = d3.scale.linear()
// 	.range([knots[3],knots[4]])
// 	.domain([margin.left, width]);

var y = d3.scale.linear()
	.domain([0,1])
	.range([height, margin.top])

var xAxis = d3.svg.axis()
	.scale(x)
	.orient("bottom")
	.ticks(4);//2*k-1)
if(!segments){
	xAxis.ticks(4)
	.tickFormat(function(d) {
	  var int = Math.floor(d);
	  if(d==domain[0]) return "0";
	  if(d==domain[1]) return "1";
	  return "."+Math.round((d-int)*10)
	 });
}

var yAxis = d3.svg.axis()
	.scale(y)
	.ticks(2)
	.orient("left")
	.tickFormat(function(d) {
	  if(d<=0) return "0";
	  if(d>=1) return "1";
	   return "."+Math.round((d)*10)
	 });

//rect x="10" y="10" width="100" height="100" stroke="blue" fill="purple"
       //fill-opacity="0.5" stroke-opacity="0.8"
if(segments){
  var segs = svg.selectAll("rect").data(segments);
  segs.enter().append("rect");

  
  segs.attr("x",function(d,i){return x(d[0])})
  .attr("y",y(1))
  .attr("height",y(0)-y(1))
  .attr("width",function(d,i){return x(d[1])-x(d[0])})
  .style("fill",color2F);
}



svg.append("g")
	.attr("class","yaxis")
	.attr("transform", "translate("+margin.left+",0)")
	.call(yAxis)
	.append("text")
	      .attr("class", "label")
	      .attr("x", width)
	      .attr("y", -6)
	      .style("text-anchor", "end")
	      .text("");

// var kontrols = svg.selectAll("circle.kontrol")
// 	.data(knots)
// 	.enter().append("circle")
// 	.style("stroke", function(d) { return color(d.i-1); })
// 	.attr("class","kontrol")
// 	.attr("r",5)
// 	.attr("cx",function(d) {return x(d);})
// 	.attr("cy",function(d) {return y(0);})
// 	.call(d3.behavior.drag()
// 	.on("dragstart", function(d) {
// 		this.__ix__ = _.indexOf(knots,d);
// 		this.__origin__ = [x(d), y(0)];
// 	})
// 	.on("drag", function(d) {
// 		knots[this.__ix__] = xinv(Math.min(x(knots[this.__ix__+1])-1, Math.max(x(knots[this.__ix__-1])+1, this.__origin__[0] += d3.event.dx)));
// 		svg.selectAll("circle.kontrol")
// 			.data(knots)
// 			.attr("cx",function(d) {return x(d);})
// 			.attr("cy",function(d) {return y(0);});

// // var basisData = d3.range(m+1).map(function(i) { return d3.range(steps+1).map(function(j) {return basisAt(i,knots[n]*j/steps);});});
// // console.log(basisData);
// var basis=svg.selectAll("path.basis")
// 	//	.data(d3.range(m+1).map(function(i) { return d3.range(steps+1).map(function(j) {return basisAt(i,knots[n]*j/steps);});}));
// 	//.data(basisData)
// 	basis.enter().append("path");
// 	basis.attr("class","basis")
// 		.attr("d",line);
// 		update();
// 	})
// 	.on("dragend", function() {
// 		delete this.__origin__;
// 	}));


function basisAt(i,t)
{
	return {"x":x(t),"y":y(bsplineBasis(i,k)(t))};
}
var steps = 300;

//bspline
var basisData = d3.range(pts).map(function(i) { 
 return {"i":i,"y":d3.range(steps+1).map(function(j) {return basisAt(i,domain[0]+(domain[1]-domain[0])*(j/steps));})};
});

//bezier
if(doBezierSpline){
var basisData = [0,1,2,3,4,5,6,7].map(function(i) {
  var ii=i;
  if(i>3){
    ii -= 1;
  }
  return {"i":ii,"y":d3.range(steps+1).map(function(j) {
      var u = j/steps;
      if(i>3){
        u += 1;
      }
      return basisAt(ii,u)
    })
  };
});
}

var basis=svg.selectAll("path.basis")
	.data(basisData);

// function makeCurve() {
// 	//curve = d3.range(steps+1).map(function(i) { return bspline(knots[k-1]+(knots[m+1]-knots[k-1])*i/steps);})
// 	var curve = [];
// 	for(var ttt=0; ttt<t-3; ttt+=0.001){
// 	  	curve.push(bspline(knots[k-1]+(knots[m+1]-knots[k-1])*ttt));
// 	}
// 	return [curve];
// }

// var i=k-1;
// var t=0;


basis.enter().append("path")
basis.attr("class","basis")
	.attr("d",function(d) { return line(d.y);})
	.style("stroke", function(d) { return color(d.i); });

// 	var basisp=svg.selectAll("circle.basisp")
// 		.data([basisAt(i-3,t),basisAt(i-2,t),basisAt(i-1,t),basisAt(i,t)]);
// 	basisp.enter().append("circle");
// 	basisp.attr("class","basisp")
// 		.attr("r",3)
// 		.attr("cx",function(d) {return d.x;})
// 		.attr("cy",function(d) {return d.y;});

svg.append("g")
	.attr("class","xaxis")
	.attr("transform", "translate(0," + height + ")")
	.call(xAxis)
	.append("text")
	      .attr("class", "label")
	      .attr("x", width+3)
	      .attr("y", 5)
	      .style("text-anchor", "start")
	      .html(timeVar); //"u"


//ImgSaver(div,svg);



}



//test code
function BernsteinBasis(i,k){
  if(k==1){
    return function(u){
      if(i==0) return 1-u;
      else return u;
    }
  }else if (k==2){
    return function(u){
      if(i==0) return (1-u)*(1-u);
      else if (i==1) return 2*u*(1-u);
      else return u*u;
    }
  }else if (k==3){
    return function(u){
      var ou = 1-u;
      if(i==0) return ou*ou*ou;
      if(i==1) return 3*ou*ou*u;
      if(i==2) return 3*ou*u*u;
      if(i==3) return u*u*u;
    }
  }
}