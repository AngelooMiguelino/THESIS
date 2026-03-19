// =======================
// INITIALIZE MAP
// =======================
const map = L.map('map').setView([14.3, 121.0], 14);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{z}/{x}/{y}.png'.replace('{z}/{z}', '{z}'), { maxZoom: 19 }).addTo(map);

// =======================
// GLOBALS
// =======================
let graph = [];
let points = [];
let blockedNodes = new Set();
let exploredLayer, routeLayer;

// =======================
// LOAD ROAD NETWORK
// =======================
async function loadRoadNetwork(center) {
  document.getElementById("metrics").innerHTML = "Loading filtered road network...";
  const query = `
  [out:json];
  (
    way
    ["highway"]
    ["highway"!~"footway|cycleway|path|pedestrian|steps|track|service|corridor|bridleway|construction"]
    (around:1200, ${center.lat}, ${center.lng});
  );
  out body;
  >;
  out skel qt;
  `;
  const res = await fetch("https://overpass-api.de/api/interpreter", { method: "POST", body: query });
  const data = await res.json();
  buildGraph(data);
  let startNode = nearestNode(points[0]);
  filterConnectedGraph(startNode);
  document.getElementById("metrics").innerHTML = "Road network ready. Select destination.";
}

// =======================
// BUILD GRAPH
// =======================
function buildGraph(data) {
  let nodes = {};
  data.elements.forEach(el => {
    if(el.type==="node") nodes[el.id]={id:el.id,lat:el.lat,lng:el.lon,neighbors:[]};
  });
  data.elements.forEach(el=>{
    if(el.type==="way"){
      let traffic = getRoadTraffic(el.tags?.highway);
      for(let i=0;i<el.nodes.length-1;i++){
        let a = nodes[el.nodes[i]], b = nodes[el.nodes[i+1]];
        if(a && b && dist(a,b)<0.005){
          a.neighbors.push({node:b,traffic});
          b.neighbors.push({node:a,traffic});
        }
      }
    }
  });
  graph = Object.values(nodes);
}

// =======================
// TRAFFIC MODEL
// =======================
function getRoadTraffic(type){
  let hour = new Date().getHours(), base = 1;
  if(type==="motorway") base=0.8;
  else if(type==="primary") base=1.0;
  else if(type==="secondary") base=1.2;
  else base=1.5;
  if(hour>=7 && hour<=9) base*=1.5;
  if(hour>=17 && hour<=19) base*=1.7;
  return base;
}

setInterval(()=>{
  graph.forEach(node=>{
    node.neighbors.forEach(edge=>{
      edge.traffic*=(0.9+Math.random()*0.2);
    });
  });
},5000);

function dist(a,b){return Math.sqrt((a.lat-b.lat)**2+(a.lng-b.lng)**2);}
function nearestNode(p){
  let best=null,min=Infinity;
  graph.forEach(n=>{let d=dist(p,n); if(d<min&&d<0.01){min=d;best=n;}});
  return best;
}
function reconstruct(prev,start,goal){
  let path=[],cur=goal;
  while(cur && cur!==start){path.push([cur.lat,cur.lng]);cur=prev.get(cur);}
  if(cur) path.push([start.lat,start.lng]);
  return path.reverse();
}

// =======================
// MAP INTERACTION
// =======================
map.on('click', async e=>{
  if(points.length>=2) resetMap();
  points.push(e.latlng);
  L.marker(e.latlng).addTo(map);
  if(points.length===1) await loadRoadNetwork(points[0]);
});
map.on('contextmenu', e=>{
  let node=nearestNode(e.latlng);
  blockedNodes.add(node);
  L.circle([node.lat,node.lng],{color:"black",radius:20}).addTo(map);
});

// =======================
// PATHFINDING ALGORITHMS
// =======================
function dijkstra(start,goal){
  let distMap=new Map(),prev=new Map(),visited=[];
  graph.forEach(n=>distMap.set(n,Infinity)); distMap.set(start,0);
  let pq=[start];
  while(pq.length){
    pq.sort((a,b)=>distMap.get(a)-distMap.get(b));
    let current=pq.shift(); visited.push(current);
    if(current===goal) break;
    current.neighbors.forEach(edge=>{
      let neighbor=edge.node;
      if(blockedNodes.has(neighbor)) return;
      let cost=distMap.get(current)+dist(current,neighbor)*edge.traffic;
      if(cost<distMap.get(neighbor)){
        distMap.set(neighbor,cost); prev.set(neighbor,current); pq.push(neighbor);
      }
    });
  }
  return {path:reconstruct(prev,start,goal),visited};
}

function astar(start,goal){
  let g=new Map(),f=new Map(),prev=new Map(),visited=[];
  graph.forEach(n=>{g.set(n,Infinity);f.set(n,Infinity);});
  g.set(start,0); f.set(start,dist(start,goal));
  let open=[start];
  while(open.length){
    open.sort((a,b)=>f.get(a)-f.get(b));
    let current=open.shift(); visited.push(current);
    if(current===goal) break;
    current.neighbors.forEach(edge=>{
      let neighbor=edge.node;
      if(blockedNodes.has(neighbor)) return;
      let temp=g.get(current)+dist(current,neighbor)*edge.traffic;
      if(temp<g.get(neighbor)){prev.set(neighbor,current); g.set(neighbor,temp); f.set(neighbor,temp+dist(neighbor,goal)); if(!open.includes(neighbor)) open.push(neighbor);}
    });
  }
  return {path:reconstruct(prev,start,goal),visited};
}

function ara(start,goal,e=2){
  let g=new Map(),f=new Map(),prev=new Map(),visited=[];
  graph.forEach(n=>{g.set(n,Infinity);f.set(n,Infinity);});
  g.set(start,0); f.set(start,e*dist(start,goal));
  let open=[start];
  while(open.length){
    open.sort((a,b)=>f.get(a)-f.get(b));
    let current=open.shift(); visited.push(current);
    if(current===goal) break;
    current.neighbors.forEach(edge=>{
      let neighbor=edge.node; if(blockedNodes.has(neighbor)) return;
      let temp=g.get(current)+dist(current,neighbor)*edge.traffic;
      if(temp<g.get(neighbor)){prev.set(neighbor,current); g.set(neighbor,temp); f.set(neighbor,temp+e*dist(neighbor,goal)); if(!open.includes(neighbor)) open.push(neighbor);}
    });
  }
  return {path:reconstruct(prev,start,goal),visited};
}

function theta(start,goal){
  let g=new Map(),parent=new Map(),visited=[];
  graph.forEach(n=>{g.set(n,Infinity); parent.set(n,null);});
  g.set(start,0); parent.set(start,start);
  let open=[start];
  while(open.length){
    open.sort((a,b)=>g.get(a)-g.get(b));
    let current=open.shift(); visited.push(current);
    if(current===goal) break;
    current.neighbors.forEach(edge=>{
      let neighbor=edge.node; if(blockedNodes.has(neighbor)) return;
      let tentative=g.get(current)+dist(current,neighbor)*edge.traffic;
      if(tentative<g.get(neighbor)){ g.set(neighbor,tentative); parent.set(neighbor,current); if(!open.includes(neighbor)) open.push(neighbor);}
    });
  }
  return {path:reconstruct(parent,start,goal),visited};
}

// =======================
// NEW ALGORITHMS
// =======================
function dstarLite(start,goal){ return astar(start,goal); } // simplified
function fieldD(start,goal){ // graph-based highway preference
  let g=new Map(),prev=new Map(),visited=[];
  graph.forEach(n=>g.set(n,Infinity));
  g.set(start,0);
  let open=[start];
  while(open.length){
    open.sort((a,b)=>g.get(a)-g.get(b));
    let current=open.shift(); visited.push(current);
    if(current===goal) break;
    current.neighbors.forEach(edge=>{
      let neighbor=edge.node; if(blockedNodes.has(neighbor)) return;
      let extra=edge.traffic<1?0.8:1; // prefer fast roads
      let temp=g.get(current)+dist(current,neighbor)*edge.traffic*extra;
      if(temp<g.get(neighbor)){ g.set(neighbor,temp); prev.set(neighbor,current); if(!open.includes(neighbor)) open.push(neighbor);}
    });
  }
  return {path:reconstruct(prev,start,goal),visited};
}
function adstar(start,goal){ return ara(start,goal,2.0); } // inflated heuristic

// =======================
// RUN ALGORITHM
// =======================
function runAlgorithm(){
  if(points.length<2){ alert("Select start and end!"); return; }
  let start=nearestNode(points[0]), goal=nearestNode(points[1]);
  let algo=document.getElementById("algorithm").value;
  let t0=performance.now(), result;

  if(algo==="dijkstra") result=dijkstra(start,goal);
  else if(algo==="astar") result=astar(start,goal);
  else if(algo==="ara") result=ara(start,goal);
  else if(algo==="theta") result=theta(start,goal);
  else if(algo==="dstar") result=dstarLite(start,goal);
  else if(algo==="fieldd") result=fieldD(start,goal);
  else if(algo==="adstar") result=adstar(start,goal);

  let t1=performance.now();
  if(routeLayer) map.removeLayer(routeLayer);
  if(exploredLayer) map.removeLayer(exploredLayer);

  exploredLayer=L.polyline(result.visited.map(n=>[n.lat,n.lng]),{color:"gray",weight:2,opacity:0.3}).addTo(map);
  routeLayer=L.polyline(result.path,{color:"red",weight:6}).addTo(map);
  map.fitBounds(routeLayer.getBounds());

  document.getElementById("metrics").innerHTML=`
    <b>Algorithm:</b> ${algo.toUpperCase()}<br>
    <b>Visited:</b> ${result.visited.length}<br>
    <b>Path Nodes:</b> ${result.path.length}<br>
    <b>Execution:</b> ${(t1-t0).toFixed(2)} ms
  `;
}

// =======================
// RESET
// =======================
function resetMap(){
  points=[]; graph=[]; blockedNodes.clear();
  map.eachLayer(layer=>{
    if(layer instanceof L.Marker || layer instanceof L.Polyline || layer instanceof L.Circle) map.removeLayer(layer);
  });
  document.getElementById("metrics").innerHTML="<b>Reset complete</b>";
}