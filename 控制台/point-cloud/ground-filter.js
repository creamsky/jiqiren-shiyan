'use strict';
// Local ground is fitted only where independent patches support a continuous surface.
// The output is a display attribute. The input XYZ and the navigation map are untouched.
function classifyGround(xyz,count,body,classes){
  classes.fill(0,0,count);
  if(!body||!Number.isFinite(body[2]))return;
  const tiles=new Map(),bins=new Map(),cell=1;
  const key=(x,y)=>x+','+y;
  for(let i=0;i<count;i++){
    const x=xyz[3*i],y=xyz[3*i+1],z=xyz[3*i+2];
    if(!Number.isFinite(z)||z<body[2]-.8||z>body[2]+.12)continue;
    const tx=Math.floor(x/cell),ty=Math.floor(y/cell),id=key(tx,ty);
    let t=tiles.get(id);
    if(!t){t={x:tx,y:ty,low:[],returns:[],plane:null,ground:false};tiles.set(id,t);}
    t.returns.push(i);
    const quadrant=(x/cell-tx>=.5?1:0)+(y/cell-ty>=.5?2:0);
    if(!t.low[quadrant]||z<t.low[quadrant][2])t.low[quadrant]=[x,y,z];
  }
  for(const t of tiles.values())for(const p of t.low){
    if(!p||Math.hypot(p[0]-body[0],p[1]-body[1])>6||p[2]>=body[2])continue;
    const bin=Math.floor(p[2]/.04);
    bins.set(bin,(bins.get(bin)||0)+1);
  }
  let anchor=null,best=0;
  for(const [bin,n] of bins){
    const support=n+.5*((bins.get(bin-1)||0)+(bins.get(bin+1)||0));
    if(support>best){best=support;anchor=(bin+.5)*.04;}
  }
  if(best<9)return;
  const neighbors=[[1,0],[-1,0],[0,1],[0,-1]];
  function floorAt(t,x,y){const p=t.plane;return p.z+p.a*(x-p.x)+p.b*(y-p.y);}
  const seeds=[];
  for(const t of tiles.values()){
    let support=[];
    for(let oy=-1;oy<=1;oy++)for(let ox=-1;ox<=1;ox++){
      const near=tiles.get(key(t.x+ox,t.y+oy));if(near)for(const p of near.low)if(p)support.push(p);
    }
    if(support.length<8)continue;
    support.sort((a,b)=>a[2]-b[2]);const median=support[support.length>>1][2];
    const n0=support.length;
    support=support.filter(p=>Math.abs(p[2]-median)<=.12);
    if(support.length<Math.max(7,n0*.65))continue;
    let x=0,y=0,z=0;
    for(const p of support){x+=p[0];y+=p[1];z+=p[2];}
    x/=support.length;y/=support.length;z/=support.length;
    let xx=0,yy=0,xy=0,xz=0,yz=0;
    for(const p of support){const dx=p[0]-x,dy=p[1]-y,dz=p[2]-z;xx+=dx*dx;yy+=dy*dy;xy+=dx*dy;xz+=dx*dz;yz+=dy*dz;}
    const det=xx*yy-xy*xy;if(det<.045*support.length*support.length)continue;
    const a=(yy*xz-xy*yz)/det,b=(xx*yz-xy*xz)/det;
    if(Math.hypot(a,b)>.22)continue;
    const good=support.filter(p=>Math.abs(p[2]-z-a*(p[0]-x)-b*(p[1]-y))<=.042).length;
    if(good<support.length*.78)continue;
    t.plane={x,y,z,a,b};
    if(Math.hypot(t.x+.5-body[0],t.y+.5-body[1])<=4&&Math.abs(floorAt(t,t.x+.5,t.y+.5)-anchor)<=.075){t.ground=true;seeds.push(t);}
  }
  for(let i=0;i<seeds.length;i++){
    const t=seeds[i];
    for(const [dx,dy] of neighbors){
      const n=tiles.get(key(t.x+dx,t.y+dy));if(!n?.plane||n.ground)continue;
      const x=t.x+.5+dx/2,y=t.y+.5+dy/2;
      if(Math.abs(floorAt(n,x,y)-floorAt(t,x,y))>.055)continue;
      n.ground=true;seeds.push(n);
    }
  }
  // Widely spaced lidar rings can leave too few low samples for a local fit.
  // Extend a measured surface only when two neighbouring planes agree and
  // independent horizontal patches in this tile actually return at that height.
  for(let pass=0;pass<3;pass++){
    const accepted=[];
    for(const t of tiles.values()){
      if(t.ground)continue;
      const x=t.x+.5,y=t.y+.5,refs=[];
      for(let dy=-2;dy<=2;dy++)for(let dx=-2;dx<=2;dx++){
        if(dx*dx+dy*dy>5)continue;
        const n=tiles.get(key(t.x+dx,t.y+dy));
        if(n?.ground)refs.push({plane:n.plane,z:floorAt(n,x,y),weight:1/(dx*dx+dy*dy)});
      }
      if(refs.length<2)continue;
      refs.sort((a,b)=>a.z-b.z);
      const median=refs[refs.length>>1].z;
      const agree=refs.filter(r=>Math.abs(r.z-median)<=.045);
      if(agree.length<2||agree.length<refs.length*.7)continue;
      let weight=0,z=0,a=0,b=0;
      for(const r of agree){weight+=r.weight;z+=r.z*r.weight;a+=r.plane.a*r.weight;b+=r.plane.b*r.weight;}
      z/=weight;a/=weight;b/=weight;
      const patches=new Set(),residual=[];
      for(const i of t.returns){
        const px=xyz[3*i],py=xyz[3*i+1],dz=xyz[3*i+2]-(z+a*(px-x)+b*(py-y));
        if(Math.abs(dz)>.04)continue;
        const patch=key(Math.floor(px*4),Math.floor(py*4));
        if(!patches.has(patch)){patches.add(patch);residual.push(dz);}
      }
      if(residual.length<3)continue;
      residual.sort((u,v)=>u-v);
      accepted.push([t,{x,y,z:z+residual[residual.length>>1],a,b}]);
    }
    if(!accepted.length)break;
    for(const [t,plane] of accepted){t.plane=plane;t.ground=true;}
  }
  // Keep a second, coherent lower surface such as a real step. Suppress an
  // isolated below-floor echo only in tiles with a locally confirmed floor.
  const below=new Map(),column=(x,y)=>key(Math.floor(x/.32),Math.floor(y/.32));
  for(const t of tiles.values())if(t.ground)for(const i of t.returns){
    const x=xyz[3*i],y=xyz[3*i+1],z=xyz[3*i+2];
    if(z>=floorAt(t,x,y)-.065)continue;
    const k=column(x,y);if(!below.has(k))below.set(k,[]);below.get(k).push([x,y,z]);
  }
  function lowerSurface(x,y,z){
    const cx=Math.floor(x/.32),cy=Math.floor(y/.32);
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
      for(const p of below.get(key(cx+dx,cy+dy))||[]){
        const d=(p[0]-x)**2+(p[1]-y)**2;
        if(d>.065**2&&d<.37**2&&Math.abs(p[2]-z)<.045)return true;
      }
    }
    return false;
  }
  function classify(x,y,z){
    const t=tiles.get(key(Math.floor(x/cell),Math.floor(y/cell)));
    if(!t?.ground)return z>anchor+2.15?4:0;
    const h=z-floorAt(t,x,y);
    if(h>=-.065&&h<=.04)return 1;
    if(h<-.065)return h>-.8&&!lowerSurface(x,y,z)?5:0;
    return h<=.90?2:h>2.15?4:3;
  }
  for(let i=0;i<count;i++)if(Number.isFinite(xyz[3*i+2]))classes[i]=classify(xyz[3*i],xyz[3*i+1],xyz[3*i+2]);
  return classify;
}
if(typeof module!=='undefined'&&module.exports)module.exports={classifyGround};
