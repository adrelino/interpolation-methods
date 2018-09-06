function BSpline(div,points,doBezierSpline,segmentOnly,continuity){
  var continuity = continuity | 0;
var pts = points.length //5 for half circle
    k=4,
    n=pts-1+k,
    //knots=[0.0, 0.000000001, 0.000000002, 0.000000003, 6.99999997, 6.99999998, 6.99999999,7.0];//d3.range(n+1),
    knots = d3.range(n+1);
//     q=[{"i":0,"x":0,"y":hh},{"i":1,"x":0,"y":0},{"i":2,"x":ww,"y":0},{"i":3,"x":ww,"y":hh}];//d3.range(m+1).map(function(i) { return {"i":i,"x":(20+i*80+Math.random()*10),"y":(Math.sin(i*4)*150+150)};}),
//     //q=d3.range(m+1).map(function(i) { return {"i":i,"x":(20+i*80+Math.random()*10),"y":(Math.sin(i*4)*150+150)};}),
//     //q=d3.range(m+1).map(function(i) { return {"i":i,"x":200+(Math.cos(i*Math.PI/2)*100),"y":200+(Math.sin(i*Math.PI/2)*100)};}),
//     for(var i=0; i<q.length; i++){
//       q[i].x += 10;
//       q[i].y += 10;
//     }
    var q = points;//.splice(0,pts);

    if(doBezierSpline){
      knots = [0,0,0,1,1,1,1];
      //C2 continuity:
    }

    var t=0,steps=100;

    var segments = pts-k-1;

var svg = d3.select(div).append("svg")
    .attr("width",ww)
    .attr("height",hh);

color = d3.scale.category10();
color2 = d3.scale.category20c().domain(d3.range(20))
//#636363 #969696 #bdbdbd #d9d9d9
color2 = d3.scale.ordinal().range(["lightgray","#9edae5","#e7cb94","#c7e9c0"])
color2F = function(d,i){
	  return color2(i);//2+4*i);
	}

var line = d3.svg.line()
	    .x(function(d) {return d.x;})
	    .y(function(d) {return d.y;})
	    .interpolate("linear");

function bsplineBasis(i,k)
{
  //bezier
  if(doBezierSpline){
    return function(tGlobal){
      var t = (tGlobal >= 1) ? tGlobal-1 : tGlobal;
      var ii=i;
      if(tGlobal >= 1) ii-=3;


      var oneMt = 1-t;
      if(ii==0) return oneMt*oneMt*oneMt;
      if(ii==1) return 3*oneMt*oneMt*t;
      if(ii==2) return 3*oneMt*t*t;
      if(ii==3) return t*t*t;

      //alert("error "+i + " "+t);
    }
  }else{
	if(k==1)
	{
		return (function(t) { if((knots[i]<=t) && (t<knots[i+1])) {return 1;} else {return 0;}});
	}
	else
	{
		return (function(t) {
		   return bsplineBasis(i,k-1)(t)*(t-knots[i])/(knots[i+k-1]-knots[i]) + bsplineBasis(i+1,k-1)(t)*(knots[i+k]-t)/(knots[i+k]-knots[i+1]);
		   //return bsplineBasis(i,k-1)(t)*(1-t) + bsplineBasis(i+1,k-1)(t)*(t); 

		});
	}
  }
}

function bspline(t,bezierSegment) 
{
	var i=k-1;
	if(doBezierSpline){
	  i = bezierSegment;
	 // i = Math.floor(t)+3;
	}else{
      while(knots[i+1]<t)
      {
          i=i+1;
      }
      if(i>pts-1) i=pts-1;
	}
	var x=0;
	var y=0;

	for(var j=0; j<k; j++){
	  var basis = bsplineBasis(i-j,k)(t);
	  var point = q[i-j];
	  x +=basis*point.x;
	  y +=basis*point.y;
	}
	//var x = bsplineBasis(i-3,k)(t)*q[i-3].x+bsplineBasis(i-2,k)(t)*q[i-2].x+bsplineBasis(i-1,k)(t)*q[i-1].x+bsplineBasis(i,k)(t)*q[i].x;
	//var y = bsplineBasis(i-3,k)(t)*q[i-3].y+bsplineBasis(i-2,k)(t)*q[i-2].y+bsplineBasis(i-1,k)(t)*q[i-1].y+bsplineBasis(i,k)(t)*q[i].y;
	return {"x":x,"y":y};
}



var time=1;
var t=knots[k-1];

// if(doAnimate){
//   function animate(current)
//   {
//         t=(t+(current-time)/10000);
//         if(t>knots[pts]) t=knots[k-1];
//         time=current;
//         update();
//   }
//   d3.timer(animate);
// }else{
  this.update = function(tt){
    t = knots[k-1]+tt;
    update();
  }
//}

svg.append("text")
  .attr("class", "tshow")
  .attr("x",ww/2) 
  .attr("y",hh-10)
  .attr("text-anchor", "middle");

//update();

function dbRecurse(i,r,t)
{
	if(r==0){
	  return q[i];
	}
 	var qq;

 	if(doBezierSpline){
      var tt = t-3;
      if(r==1){
        qq={
          "r":r,
          "x":((1-tt)*q[i-1].x + tt*q[i].x),
          "y":((1-tt)*q[i-1].y + tt*q[i].y)
        };
        return qq;
      }else if (r==2){
        qq={
          "r":r,
          "x":((1-tt)*dbRecurse(i-1,r-1,t).x+tt*dbRecurse(i,r-1,t).x),
          "y":((1-tt)*dbRecurse(i-1,r-1,t).y+tt*dbRecurse(i,r-1,t).y)
        };
      }
 	}else{
 	  var a=(t-knots[i])/(knots[i+k-r]-knots[i]);
      qq={
        "r":r,
        "x":((1-a)*dbRecurse(i-1,r-1,t).x+a*dbRecurse(i,r-1,t).x),
        "y":((1-a)*dbRecurse(i-1,r-1,t).y+a*dbRecurse(i,r-1,t).y)
      };
 	}
	return qq;
}


function dB(t)
{
	i=k-1;
	while(knots[i+1]<t)
	{
		i=i+1;
	}
	if(i>pts-1) i=pts-1;
	if(k==2) return [];
	if(k==3) return [[dbRecurse(i-1,1,t),dbRecurse(i,1,t)]];
	return [[dbRecurse(i-2,1,t),dbRecurse(i-1,1,t),dbRecurse(i,1,t)],[dbRecurse(i-1,2,t),dbRecurse(i,2,t)]];
}

//var continuity = 2; //1 or 2
var endConditionCont = 2; //derivative at 


function enforeContinuity(d,i){
  if(!doBezierSpline) return;

  var xD = d.x-d.xOld;
  var yD = d.y-d.yOld;

    if(endConditionCont>0){
      if(i==0){
        q[1].x += xD;
        q[1].y += yD;
      }else if(i==6){
        q[5].x += xD;
        q[5].y += yD;
      }
    }

    if(continuity>0){

      if(i==2){
         q[4].x=q[3].x+(q[3].x-d.x);
         q[4].y=q[3].y+(q[3].y-d.y);
      }else if(i==3){
         q[4].x+=xD;
         q[4].y+=yD;
         q[2].x+=xD;
         q[2].y+=yD;
      }else if(i==4){
         q[2].x=q[3].x+(q[3].x-d.x);
         q[2].y=q[3].y+(q[3].y-d.y);
      }

      if(continuity==2){
        var diff = {x:0, y: 0};
        diff.x=q[3].x-q[2].x;
        diff.y=q[3].y-q[2].y;

        if(i==3 && endConditionCont==2){
         q[5].x+=xD/2;
         q[5].y+=yD/2;
         q[1].x+=xD/2;
         q[1].y+=yD/2
        }

        if(i==1){
           q[5].x=q[1].x+4*diff.x;
           q[5].y=q[1].x+4*diff.y;
        }else if(i==2){
           q[5].x=q[1].x+4*diff.x;
           q[5].y=q[1].y+4*diff.y;
        }else if(i==4){
           q[1].x=q[5].x-4*diff.x;
           q[1].y=q[5].y-4*diff.y;
        }else if(i==5){
           q[1].x=q[5].x-4*diff.x;
           q[1].y=q[5].x-4*diff.y;
        }
      }
     }
}


function update() {
	/*var db=dB(t);
	var deB1=svg.selectAll("path.db1")
		.data(db);
	deB1.enter().append("path");
	deB1.attr("class","db1")
		.attr("d",line)
		.style("stroke", function(d) { return "gray"; });*/

			//enforeContinuity(q[2],2);


    var hull=svg.selectAll("path.hull")
		.data([q]);
	hull.enter().append("path");
	hull.attr("class","hull")
		.attr("d",line);
	
	/*var deB2=svg.selectAll("circle.db2")
		.data(_.flatten(db));
	deB2.enter().append("circle");
	deB2.attr("class","db2")
		.attr("r",3)
		.attr("cx",function(d) {return d.x;})
		.attr("cy",function(d) {return d.y;})
		//.style("stroke", function(d) { return "black"; })
		.style("fill", function(d) { return "black"; });*/

	var curveGraph=svg.selectAll("path.curve")
		.data(makeCurve());
	curveGraph.enter().append("path");
	curveGraph.attr("class","curve")
		.attr("d",line);
	curveGraph.style("stroke",color2F)


/*	i=k-1;
	while(knots[i+1]<t)
	{
		i=i+1;
	}
	var point=svg.selectAll("circle.point")
		.data([bspline(t)]);
	point.enter().append("circle");
	point.attr("class","point")
		.attr("r",3)
		.attr("cx",function(d) {return d.x;})
		.attr("cy",function(d) {return d.y;});*/


	//svg.selectAll("text.tshow")
		//      .html("u = " + (t-knots[i]).toFixed(2) + ", &#964 ="+ t);
		      


var controls = svg.selectAll("circle.control")
	.data(q)
	.enter().append("circle")
	.attr("class","control")
	.attr("r",7)
	.attr("cx",function(d) {return d.x;})
	.attr("cy",function(d) {return d.y;})
	.style("fill", function(d,i) { return color(i); })
	.call(d3.behavior.drag()
	.on("dragstart", function(d,i) {
	  console.log(i);
		this.__origin__ = [d.x, d.y];
	})
	.on("drag", function(d,i) {
	    d.xOld = d.x;
	    d.yOld = d.y;
	  	d.x = Math.min(ww, Math.max(0, this.__origin__[0] += d3.event.dx));
		d.y = Math.min(400, Math.max(0, this.__origin__[1] += d3.event.dy));

		enforeContinuity(d,i);



		//console.log(d);



		svg.selectAll("circle.control")
			.data(q)
			.attr("cx",function(d) {return d.x;})
			.attr("cy",function(d) {return d.y;});
		update();
	})
	.on("dragend", function(d,i) {
		delete this.__origin__;
	}));

// svg.selectAll("text.controltext")
//     .data(q)
//   .enter().append("text")
//     .attr("class", "controltext")
//     .attr("dx", "0px")
//     .attr("dy", ".4em")
//     .attr("cx",function(d) {return d.x;})
// 	.attr("cy",function(d) {return d.y;})
//     .attr("text-anchor", "middle")
//     .text(function(d, i) { return i });
}

var doBasis=true;

function makeCurve() {
	//curve = d3.range(steps+1).map(function(i) { return bspline(knots[k-1]+(knots[m+1]-knots[k-1])*i/steps);})
	var curves = [];
	var segments = [];
	if(doBezierSpline){
      for(var j=0; j<2; j++){
        var curve = [];
        for(var ttt=j; ttt<j+1; ttt+=0.01){
            curve.push(bspline(ttt,j==0 ? 3 : 6));
        }
        segments.push([j,j+1]);
        curves.push(curve);
      }
	}else{
      for(var j=k-1; j<pts; j++){
        var curve = [];
        for(var ttt=j; ttt<j+1; ttt+=0.01){
            curve.push(bspline(ttt));
        }
        segments.push([j,j+1]);
        curves.push(curve);
      }
	}
	if(doBasis){
	  	plotBasis(div,knots,bsplineBasis,k,segments,doBezierSpline);
	  	doBasis=false;
	}
	return curves;

}

this.setContinuity = function(c){
  console.log("setContinuity", c);
  continuity = c;
  if(continuity>=1){
  points[4].x=points[3].x+(points[3].x-points[2].x);
  points[4].y=points[3].y+(points[3].y-points[2].y);
  }
  
  if(continuity>=2){
  points[2].x=(points[1].x+points[3].x)/2;
  points[2].y=(points[1].y+points[3].y)/2;
  points[4].x=(points[5].x+points[3].x)/2;
  points[4].y=(points[5].y+points[3].y)/2;
  }
  
//   for(var i=0; i<7; i++){
//   enforeContinuity(points[i]);
//   }
		svg.selectAll("circle.control")
			.data(q)
			.attr("cx",function(d) {return d.x;})
			.attr("cy",function(d) {return d.y;});
  update();
}


//ImgSaver(div,svg);

};