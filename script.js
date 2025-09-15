/*
 * Extracted JavaScript from the original index_last.html. This file
 * encapsulates all logic for the interactive graph, handling user
 * interactions, drawing routines, and state management. The code is
 * preserved as an immediately-invoked function expression (IIFE) to
 * avoid polluting the global namespace. Any new behavior should be
 * added below or inside this closure to ensure it executes after
 * DOM elements are available.
 */

(function(){
  const $ = s => document.querySelector(s);
  const canvas = $('#canvas'), ctx = canvas.getContext('2d');

  // Mode buttons
  const modeLineBtn = $('#modeLine'), modeDataBtn = $('#modeData');

  // Sections
  const lineControls = $('#lineControls');
  const dataControls = $('#dataControls');

  // LINE controls
  const slope = $('#slope'), shift = $('#shift');
  const segRadios = Array.from(document.querySelectorAll('input[name="segmode"]'));
  const fixedRow = $('#fixedRow'); const xMin = $('#xMin'); const xMax = $('#xMax');

  // DATA controls
  const px = $('#px'), py = $('#py'), addPoint = $('#addPoint'),
        undoPoint = $('#undoPoint'), clearPoints = $('#clearPoints'),
        csvBox = $('#csvBox'), importCSV = $('#importCSV'), fitDataBtn = $('#fitData'),
        pointList = $('#pointList');

  // Custom axes controls
  const customAxes = $('#customAxes'), customAxesSection = $('#customAxesSection');
  const xAxisMin = $('#xAxisMin'), xAxisMax = $('#xAxisMax'), xAxisInc = $('#xAxisInc'), xLabelNth = $('#xLabelNth');
  const xUseTickIndex = $('#xUseTickIndex'), xTickIdxMin = $('#xTickIdxMin'), xTickIdxMax = $('#xTickIdxMax');
  const yAxisMin = $('#yAxisMin'), yAxisMax = $('#yAxisMax'), yAxisInc = $('#yAxisInc'), yLabelNth = $('#yLabelNth');
  const yUseTickIndex = $('#yUseTickIndex'), yTickIdxMin = $('#yTickIdxMin'), yTickIdxMax = $('#yTickIdxMax');
  const xAxisLabel = $('#xAxisLabel'), yAxisLabel = $('#yAxisLabel');
  const viewPadPct = $('#viewPadPct');

  const crosshair = $('#crosshair'), markers = $('#markers');
  const showArea = $('#showArea'); // <-- Add this line
  const fitAll = $('#fitAll'), zoomIn = $('#zoomIn'), zoomOut = $('#zoomOut'),
        resetView = $('#resetView'), exportBtn = $('#export'), resetLabelPos = $('#resetLabelPos');

  // Readouts
  const ro_m = $('#ro_m'), ro_h = $('#ro_h'), ro_seg = $('#ro_seg');
  const ro_ap = $('#ro_ap'), ro_an = $('#ro_an'), ro_net = $('#ro_net');
  const ro_units_raw = $('#ro_units_raw'),
        ro_units_simple = $('#ro_units_simple'),
        ro_units_simple2 = $('#ro_units_simple2'),
        ro_units_simple3 = $('#ro_units_simple3');
  const ro_xint = $('#ro_xint'), ro_yint = $('#ro_yint'), ro_xu = $('#ro_xu'), ro_yu = $('#ro_yu');
  const ro_cursor = $('#ro_cursor'), cx = $('#cx'), cy = $('#cy'), ro_gtype = $('#ro_gtype');

  // State
  const state = {
    graphType: 'line',
    m: +slope.value, h: +shift.value,
    segMode: segRadios.find(r=>r.checked).value,
    xMin: +xMin.value, xMax: +xMax.value,
    points: [],
    scaleX: 56, scaleY: 56, tx: 0, ty: 0,
    customAxes: false,
    xAxisMin: +xAxisMin.value, xAxisMax: +xAxisMax.value, xAxisInc: +xAxisInc.value, xLabelNth: +xLabelNth.value,
    yAxisMin: +yAxisMin.value, yAxisMax: +yAxisMax.value, yAxisInc: +yAxisInc.value, yLabelNth: +yLabelNth.value,
    xUseTickIndex: false, xTickIdxMin: +xTickIdxMin.value, xTickIdxMax: +xTickIdxMax.value,
    yUseTickIndex: false, yTickIdxMin: +yTickIdxMin.value, yTickIdxMax: +yTickIdxMax.value,
    viewPad: ((+viewPadPct?.value)||8)/100,
    xLabel: xAxisLabel.value.trim() || 'x',
    yLabel: yAxisLabel.value.trim() || 'y',
    showCrosshair: false, showMarkers: true,
    showArea: true, // <-- Add this line

    // NEW: draggable label offsets (screen-space, px)
    labelOffsetX: {dx:0, dy:0},
    labelOffsetY: {dx:0, dy:0},
  };

  // label hit regions (screen-space)
  let labelRects = { x:{x:0,y:0,w:0,h:0}, y:{x:0,y:0,w:0,h:0} };

  // ---------- Canvas & transforms ----------
  function fitCanvas(){
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const rect = canvas.getBoundingClientRect();
    const needW = Math.round(rect.width * dpr);
    const needH = Math.round(rect.height * dpr);
    if (canvas.width !== needW || canvas.height !== needH){
      canvas.width = needW; canvas.height = needH;
    }
    ctx.setTransform(dpr,0,0,dpr,0,0);
  }
  function initCamera(){
    const rect = canvas.getBoundingClientRect();
    state.tx = rect.width/2;
    state.ty = rect.height/2;
  }
  function toScreen(x,y){ return [x*state.scaleX + state.tx, -y*state.scaleY + state.ty]; }
  function toWorld(xs,ys){ return [(xs - state.tx)/state.scaleX, -(ys - state.ty)/state.scaleY]; }
  function xWorldSpan(){ const r=canvas.getBoundingClientRect(); return [toWorld(0,0)[0], toWorld(r.width,0)[0]]; }
  function yWorldSpan(){ const r=canvas.getBoundingClientRect(); return [toWorld(0,r.height)[1], toWorld(0,0)[1]]; }

  // ---------- Helpers ----------
  function formatNum(v, forceDecimals = false) {
    if (!isFinite(v)) return '—';
    if (forceDecimals) {
      if (Math.abs(v) < 1) return v.toFixed(2);
      if (Math.abs(v) < 10) return v.toFixed(1);
      return v.toFixed(0);
    }
    const val = +v;
    if (val === parseInt(val, 10)) return val.toString();
    return val.toFixed(6).replace(/(\.\d*?[1-9])0+$/,'$1').replace(/\.0+$/,'');
  }
  function getVar(name){ return getComputedStyle(document.documentElement).getPropertyValue(name); }
  function niceStep(pixelsPerUnit, targetPx=60){
    const raw = targetPx / Math.max(1e-9,pixelsPerUnit);
    const pow = Math.pow(10, Math.floor(Math.log10(raw)));
    const mult = raw / pow;
    const choice = (mult<1.5)?1:(mult<3.5)?2:5;
    return choice*pow;
  }
  function yLine(x){ return state.m * (x - state.h); }

  // Units (trimmed)
  function parseUnit(u){
    u=(u||'').trim(); if(!u) return {num:new Map(),den:new Map()};
    u=u.replace(/\*/g,'·').replace(/·+/g,'·').replace(/\s+/g,' ').trim();
    const [A,B=''] = u.split('/');
    const parse=(s)=>{ const m=new Map(); if(!s) return m;
      s.split(/[·\s]+/).forEach(t=>{
        if(!t) return; const r=t.trim().match(/^([A-Za-zμΩ]+)(?:\^(-?\d+))?$/);
        if(r){ m.set(r[1],(m.get(r[1])||0)+(r[2]?+r[2]:1)); } else { m.set(t,(m.get(t)||0)+1); }
      }); return m; };
    return {num:parse(A), den:parse(B)};
  }
  function multiplyUnits(Ua,Ub){
    const out={num:new Map(),den:new Map()}; const add=(dst,src,s=1)=>src.forEach((v,k)=>dst.set(k,(dst.get(k)||0)+s*v));
    add(out.num,Ua.num,1); add(out.den,Ua.den,1); add(out.num,Ub.num,1); add(out.den,Ub.den,1);
    out.num.forEach((e,k)=>{ const d=out.den.get(k)||0, net=e-d;
      if(net===0){ out.num.delete(k); out.den.delete(k); }
      else if(net>0){ out.num.set(k,net); out.den.delete(k); }
      else{ out.num.delete(k); out.den.set(k,-net); }});
    return out;
  }
  function unitToString(U){
    const side=(m)=>{ const pr=['m','s','kg','N','J']; const ks=[...m.keys()].sort((a,b)=>{const ia=pr.indexOf(a),ib=pr.indexOf(b);return (ia===-1?99:ia)-(ib===-1?99:ib)||a.localeCompare(b)}); return ks.map(k=>{const e=m.get(k);return e===1?k:`${k}^${e}`}).join('·'); };
    const a=side(U.num), b=side(U.den); if(b&&a) return a+'/'+b; if(b&&!a) return '1/'+b; return a||'1';
  }
  function simplifyAreaUnits(yLabel,xLabel){
    const s = unitToString(multiplyUnits(parseUnit(yLabel), parseUnit(xLabel)));
    return (s==='N·m')?'J':s;
  }

  // ---------- Custom domain & padded view ----------
  function computeCustomDomainAndView(){
    const rect = canvas.getBoundingClientRect();
    let stepX = Math.max(1e-12, Math.abs(+state.xAxisInc) || 1);
    let stepY = Math.max(1e-12, Math.abs(+state.yAxisInc) || 1);

    const idxXmin = Math.min(state.xTickIdxMin, state.xTickIdxMax);
    const idxXmax = Math.max(state.xTickIdxMin, state.xTickIdxMax);
    const idxYmin = Math.min(state.yTickIdxMin, state.yTickIdxMax);
    const idxYmax = Math.max(state.yTickIdxMin, state.yTickIdxMax);

    const xMin = isFinite(state.xUseTickIndex ? idxXmin * stepX : +state.xAxisMin) ? (state.xUseTickIndex ? idxXmin * stepX : +state.xAxisMin) : -10;
    const xMax = isFinite(state.xUseTickIndex ? idxXmax * stepX : +state.xAxisMax) ? (state.xUseTickIndex ? idxXmax * stepX : +state.xAxisMax) : 10;
    const yMin = isFinite(state.yUseTickIndex ? idxYmin * stepY : +state.yAxisMin) ? (state.yUseTickIndex ? idxYmin * stepY : +state.yAxisMin) : -10;
    const yMax = isFinite(state.yUseTickIndex ? idxYmax * stepY : +state.yAxisMax) ? (state.yUseTickIndex ? idxYmax * stepY : +state.yAxisMax) : 10;

    const xRange = Math.max(1e-9, xMax - xMin);
    const yRange = Math.max(1e-9, yMax - yMin);

    const pad = Math.max(0, Math.min(0.40, state.viewPad ?? 0.08));
    const xMinView = xMin - pad * xRange;
    const xMaxView = xMax + pad * xRange;
    const yMinView = yMin - pad * yRange;
    const yMaxView = yMax + pad * yRange;

    // Camera from padded view box
    state.scaleX = rect.width  / Math.max(1e-9, xMaxView - xMinView);
    state.scaleY = rect.height / Math.max(1e-9, yMaxView - yMinView);
    state.tx = -xMinView * state.scaleX;
    state.ty = rect.height + yMinView * state.scaleY;

    return {xMin,xMax,yMin,yMax, xMinView,xMaxView,yMinView,yMaxView, stepX,stepY};
  }

  // ---------- Grid & axes ----------
  function drawAxisArrow(x,y,rot){
    ctx.save(); ctx.translate(x,y); ctx.rotate(rot);
    ctx.fillStyle = getVar('--axis'); ctx.beginPath();
    ctx.moveTo(0,0); ctx.lineTo(-10,3.5); ctx.lineTo(-10,-3.5); ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  function drawGrid(){
    const rect = canvas.getBoundingClientRect();
    ctx.fillStyle = getVar('--panel'); ctx.fillRect(0,0,rect.width,rect.height);

    let xTicksToDraw = [], yTicksToDraw = [];
    let stepX, stepY;
    let dom = null;

    if (state.customAxes){
      dom = computeCustomDomainAndView();
      stepX = dom.stepX; stepY = dom.stepY;

      // Anchor ticks to domain start, extend through padded view
      const MAX_TICKS = 2000;
      const kStartX = Math.floor((dom.xMinView - dom.xMin) / stepX);
      const kEndX   = Math.ceil ((dom.xMaxView - dom.xMin) / stepX);
      let kStepX = 1;
      if (kEndX - kStartX + 1 > MAX_TICKS) kStepX = Math.ceil((kEndX - kStartX + 1) / MAX_TICKS);
      for (let k = kStartX; k <= kEndX; k += kStepX){ xTicksToDraw.push(dom.xMin + k * stepX); }

      const kStartY = Math.floor((dom.yMinView - dom.yMin) / stepY);
      const kEndY   = Math.ceil ((dom.yMaxView - dom.yMin) / stepY);
      let kStepY = 1;
      if (kEndY - kStartY + 1 > MAX_TICKS) kStepY = Math.ceil((kEndY - kStartY + 1) / MAX_TICKS);
      for (let k = kStartY; k <= kEndY; k += kStepY){ yTicksToDraw.push(dom.yMin + k * stepY); }
    } else {
      const targetGridPx = 60;
      const xSpan = Math.abs(xWorldSpan()[1] - xWorldSpan()[0]);
      const ySpan = Math.abs(yWorldSpan()[1] - yWorldSpan()[0]);
      stepX = niceStep(rect.width / xSpan, targetGridPx);
      stepY = niceStep(rect.height / ySpan, targetGridPx);

      const [xl,xr]=xWorldSpan(), x0=Math.floor(xl/stepX)*stepX;
      for(let x=x0;x<=xr+1e-9;x+=stepX){ xTicksToDraw.push(x); }
      const [yb,yt]=yWorldSpan(), y0=Math.floor(yb/stepY)*stepY;
      for(let y=y0;y<=yt+1e-9;y+=stepY){ yTicksToDraw.push(y); }
    }
    
    // Major grid lines
    ctx.strokeStyle = getVar('--grid-major'); ctx.lineWidth = 1.25; ctx.beginPath();
    for(const x of xTicksToDraw){ const [sx]=toScreen(x,0); ctx.moveTo(sx,0); ctx.lineTo(sx,rect.height); }
    for(const y of yTicksToDraw){ const [,sy]=toScreen(0,y); ctx.moveTo(0,sy); ctx.lineTo(rect.width,sy); }
    ctx.stroke();

    // Axes + arrows
    ctx.strokeStyle = getVar('--axis'); ctx.lineWidth = 2.2;
    ctx.beginPath(); ctx.moveTo(0,state.ty); ctx.lineTo(rect.width,state.ty);
    ctx.moveTo(state.tx,0); ctx.lineTo(state.tx,rect.height); ctx.stroke();
    drawAxisArrow(rect.width-8, state.ty, 0);
    drawAxisArrow(state.tx, 8, -Math.PI/2);

    // Tick labels
    ctx.fillStyle = getVar('--muted'); ctx.font='12px Inter, system-ui, sans-serif';

    // X tick labels
    {
      const xNth = Math.max(1, (state.xLabelNth|0) || 1);
      const eps = 1e-9;
      for(const x of xTicksToDraw){
        let labelIt = true, iDom = 0;
        if (state.customAxes){
          const inDom = (x >= (dom?.xMin ?? -Infinity) - eps && x <= (dom?.xMax ?? Infinity) + eps);
          iDom = Math.round((x - (dom?.xMin ?? 0)) / (stepX||1));
          labelIt = inDom && (iDom % xNth === 0);
        }
        if (!labelIt) continue;
        if (Math.abs(x) < 1e-12) continue;
        const [sx] = toScreen(x,0);
        ctx.textAlign='center'; ctx.textBaseline='bottom';
        ctx.fillText(formatNum(x, Math.abs(stepX) < 1), sx, state.ty-4);
      }
    }
    // Y tick labels
    {
      const yNth = Math.max(1, (state.yLabelNth|0) || 1);
      const eps = 1e-9;
      for(const y of yTicksToDraw){
        let labelIt = true, iDom = 0;
        if (state.customAxes){
          const inDom = (y >= (dom?.yMin ?? -Infinity) - eps && y <= (dom?.yMax ?? Infinity) + eps);
          iDom = Math.round((y - (dom?.yMin ?? 0)) / (stepY||1));
          labelIt = inDom && (iDom % yNth === 0);
        }
        if (!labelIt) continue;
        if (Math.abs(y) < 1e-12) continue;
        const [,sy] = toScreen(0,y);
        ctx.textAlign='right'; ctx.textBaseline='middle';
        ctx.fillText(formatNum(y, Math.abs(stepY) < 1), state.tx-6, sy);
      }
    }

    // Axis unit labels (DRAGGABLE)
    ctx.fillStyle = getVar('--muted'); ctx.font='13px Inter, system-ui, sans-serif';

    // --- X label ---
    const xDefaultX = rect.width - 8;
    const xDefaultY = Math.min(rect.height-4, Math.max(12, state.ty-6));
    const xPosX = xDefaultX + state.labelOffsetX.dx;
    const xPosY = xDefaultY + state.labelOffsetX.dy;

    ctx.textAlign='right'; ctx.textBaseline='bottom';
    ctx.fillText(state.xLabel, xPosX, xPosY);

    // measure & store hit rect for X (axis-aligned, generous padding)
    const xMetrics = ctx.measureText(state.xLabel || '');
    const xW = Math.max(24, (xMetrics.width||0) + 8);
    labelRects.x = { x: xPosX - xW, y: xPosY - 16, w: xW, h: 20 };

    // --- Y label (vertical, centered) ---
    const clamp = (v,min,max)=>Math.max(min,Math.min(max,v));
    const yDefaultX = clamp(state.tx+12, 12, rect.width-12);
    const yDefaultY = rect.height/2;
    const yPosX = yDefaultX + state.labelOffsetY.dx;
    const yPosY = yDefaultY + state.labelOffsetY.dy;

    ctx.save();
    ctx.translate(yPosX, yPosY);
    ctx.rotate(-Math.PI/2);
    ctx.textAlign='center';
    ctx.textBaseline='middle';
    ctx.fillText(state.yLabel, 0, 0);
    ctx.restore();

    // measure & store hit rect for Y (approx vertical box)
    const yMetrics = ctx.measureText(state.yLabel || '');
    const yH = Math.max(28, (yMetrics.width||0) + 12); // height along vertical direction
    labelRects.y = { x: yPosX - 12, y: yPosY - yH/2, w: 24, h: yH };
  }

  // ---------- Area + intercepts (LINE) ----------
  function areaPolysLine(xa,xb,samples=600){
    if (xa>xb) [xa,xb]=[xb,xa];
    const dx=(xb-xa)/samples; const pos=[],neg=[]; let x0=xa,y0=yLine(x0);
    for(let i=1;i<=samples;i++){
      const x1=xa+i*dx, y1=yLine(x1);
      if ((y0>=0&&y1>=0)||(y0<=0&&y1<=0)){
        (y0>=0?pos:neg).push([[x0,y0],[x1,y1]]);
      }else{
        const t=(0-y0)/(y1-y0), xc=x0+t*(x1-x0);
        if (y0>0){ pos.push([[x0,y0],[xc,0]]); neg.push([[xc,0],[x1,y1]]); }
        else     { neg.push([[x0,y0],[xc,0]]); pos.push([[xc,0],[x1,y1]]); }
      }
      x0=x1; y0=y1;
    }
    return stripsToPolys(pos,neg);
  }
  function areasLine(xa,xb){
    if (xa>xb) [xa,xb]=[xb,xa];
    const m=state.m,h=state.h, F=x=>0.5*m*x*x - m*h*x;
    const cuts=[xa, ...(h>xa&&h<xb?[h]:[]), xb];
    let Apos=0,Aneg=0;
    for (let i=0;i<cuts.length-1;i++){
      const a=cuts[i], b=cuts[i+1], mid=(a+b)/2, s=Math.sign(yLine(mid)), A=F(b)-F(a);
      if (s>=0) Apos+=Math.abs(A); else Aneg+=Math.abs(A);
    }
    return {Apos,Aneg,Anet:Apos-Aneg};
  }
  function lineSegmentRange(){
    if (state.segMode==='fixed'){ const a=Math.min(state.xMin,state.xMax), b=Math.max(state.xMin,state.xMax); return [a,b]; }
    if (state.segMode==='axes'){ const xh=state.h; return [Math.min(0,xh), Math.max(0,xh)]; }
    return xWorldSpan();
  }
  function lineIntercepts(){
    const m=state.m,h=state.h;
    const xints = (Math.abs(m)<1e-12)?['∞ (on x-axis)']:[formatNum(h)];
    const yint  = (Math.abs(m)<1e-12)?'0':formatNum(-m*h);
    return {xints, yint};
  }

  // ---------- Area + intercepts (DATA) ----------
  function areaPolysData(){
    const P = state.points; if (P.length<2) return {posPoly:[],negPoly:[]};
    const posStrips=[], negStrips=[];
    for (let i=0;i<P.length-1;i++){
      let x0=P[i].x, y0=P[i].y, x1=P[i+1].x, y1=P[i+1].y;
      if (x1===x0) continue;
      if (y0===0 && y1===0){
      } else if ((y0>=0&&y1>=0)||(y0<=0&&y1<=0)){
        (y0>=0?posStrips:negStrips).push([[x0,y0],[x1,y1]]);
      } else {
        const m=(y1-y0)/(x1-x0), xc = x0 - y0/m;
        (y0>0?posStrips:negStrips).push([[x0,y0],[xc,0]]);
        (y0>0?negStrips:posStrips).push([[xc,0],[x1,y1]]);
      }
    }
    return stripsToPolys(posStrips, negStrips);
  }
  function areasData(){
    const P = state.points; if (P.length<2) return {Apos:0,Aneg:0,Anet:0};
    let Apos=0,Aneg=0;
    for (let i=0;i<P.length-1;i++){
      const a=P[i], b=P[i+1]; if (a.x===b.x) continue;
      const segArea = (x0,y0,x1,y1)=>{
        const A = (y0+y1)/2 * (x1-x0);
        const sign = Math.sign((y0+y1)/2 || (y0||y1));
        if (sign>=0) Apos += Math.abs(A); else Aneg += Math.abs(A);
      };
      if (a.y===0 || b.y===0 || (a.y>0&&b.y>0) || (a.y<0&&b.y<0)){
        segArea(a.x,a.y,b.x,b.y);
      } else {
        const m=(b.y-a.y)/(b.x-a.x), xc = a.x - a.y/m;
        segArea(a.x,a.y,xc,0);
        segArea(xc,0,b.x,b.y);
      }
    }
    return {Apos,Aneg,Anet:Apos-Aneg};
  }
  function dataIntercepts(){
    const P = state.points;
    const xints=[];
    for (let i=0;i<P.length-1;i++){
      const a=P[i], b=P[i+1];
      if (a.y===0) xints.push(a.x);
      if ((a.y>0&&b.y<0)||(a.y<0&&b.y>0)){
        const m=(b.y-a.y)/(b.x-a.x), xc = a.x - a.y/m; xints.push(xc);
      }
      if (b.y===0) xints.push(b.x);
    }
    let yint = null;
    for (let i=0;i<P.length-1;i++){
      const a=P[i], b=P[i+1];
      const minx=Math.min(a.x,b.x), maxx=Math.max(a.x,b.x);
      if (0>=minx && 0<=maxx && a.x!==b.x){
        const t = (0 - a.x)/(b.x - a.x);
        yint = a.y + t*(b.y - a.y);
        break;
      }
    }
    const xs = [...xints].filter(x=>isFinite(x)).sort((u,v)=>u-v);
    const dedup=[]; const eps=1e-9;
    for (const x of xs){ if (!dedup.length || Math.abs(x-dedup[dedup.length-1])>eps) dedup.push(x); }
    return {xints: dedup, yint};
  }
  function stripsToPolys(posStrips, negStrips){
    const toPoly = strips => {
      if (!strips.length) return [];
      const top=[]; for (const s of strips){ top.push(s[0],s[1]); }
      const base=[]; for (let i=top.length-1;i>=0;i--){ base.push([top[i][0],0]); }
      return top.concat(base);
    };
    return {posPoly:toPoly(posStrips), negPoly:toPoly(negStrips)};
  }

  // ---------- Drawing ----------
  function drawPoly(poly, fillStyle){
    if (poly.length<3) return;
    ctx.beginPath();
    for (let i=0;i<poly.length;i++){
      const [x,y]=poly[i]; const [sx,sy]=toScreen(x,y);
      if (i===0) ctx.moveTo(sx,sy); else ctx.lineTo(sx,sy);
    }
    ctx.closePath(); ctx.fillStyle=fillStyle; ctx.fill();
  }
  function drawLineSegment(xa,xb){
    const [sx1,sy1]=toScreen(xa,yLine(xa));
    const [sx2,sy2]=toScreen(xb,yLine(xb));
    ctx.strokeStyle=getVar('--accent'); ctx.lineWidth=2.2;
    ctx.beginPath(); ctx.moveTo(sx1,sy1); ctx.lineTo(sx2,sy2); ctx.stroke();
    ctx.fillStyle=getVar('--accent');
    ctx.beginPath(); ctx.arc(sx1,sy1,3,0,2*Math.PI); ctx.fill();
    ctx.beginPath(); ctx.arc(sx2,sy2,3,0,2*Math.PI); ctx.fill();
  }
  function drawPolyline(){
    const P = state.points; if (P.length===0) return;
    ctx.strokeStyle=getVar('--accent'); ctx.lineWidth=2.2; ctx.beginPath();
    for (let i=0;i<P.length;i++){
      const [sx,sy]=toScreen(P[i].x,P[i].y);
      if (i===0) ctx.moveTo(sx,sy); else ctx.lineTo(sx,sy);
    }
    ctx.stroke();
    ctx.fillStyle=getVar('--accent');
    for (let i=0;i<P.length;i++){ const [sx,sy]=toScreen(P[i].x,P[i].y);
      ctx.beginPath(); ctx.arc(sx,sy,3,0,2*Math.PI); ctx.fill(); }
  }
  function drawInterceptMarkersLine(){
    if (!state.showMarkers) return;
    const m=state.m,h=state.h; ctx.fillStyle=getVar('--fg'); ctx.font='12px Inter, system-ui, sans-serif';
    const yint = -m*h; { const [sx,sy]=toScreen(0,yint); dot(sx,sy); label(sx+6,sy-6,`y=${formatNum(yint)}`); }
    if (Math.abs(m)>=1e-12){ const xint=h; const [sx,sy]=toScreen(xint,0); dot(sx,sy); label(sx+6,sy-6,`x=${formatNum(xint)}`); }
    function dot(x,y){ ctx.beginPath(); ctx.arc(x,y,3,0,2*Math.PI); ctx.fill(); }
    function label(x,y,t){ ctx.fillText(t,x,y); }
  }
  function drawInterceptMarkersData(){
    if (!state.showMarkers || state.points.length<2) return;
    const {xints,yint} = dataIntercepts();
    ctx.fillStyle=getVar('--fg'); ctx.font='12px Inter, system-ui, sans-serif';
    if (yint!==null && isFinite(yint)){ const [sx,sy]=toScreen(0,yint); dot(sx,sy); label(sx+6,sy-6,`y=${formatNum(yint)}`); }
    xints.slice(0,6).forEach(xc=>{ const [sx,sy]=toScreen(xc,0); dot(sx,sy); label(sx+6,sy-6,`x=${formatNum(xc)}`); });
    function dot(x,y){ ctx.beginPath(); ctx.arc(x,y,3,0,2*Math.PI); ctx.fill(); }
    function label(x,y,t){ ctx.fillText(t,x,y); }
  }

  // ---------- Render ----------
  function render(){
    fitCanvas();
    drawGrid(); // sets labelRects

    const rawUnits = (state.yLabel||'y') + '·' + (state.xLabel||'x');
    const simp = simplifyAreaUnits(state.yLabel, state.xLabel);

    if (state.graphType==='line'){
      const [xa,xb] = lineSegmentRange();
      const {posPoly,negPoly} = areaPolysLine(xa,xb);
      if (state.showArea) { // <-- Only draw area if enabled
      drawPoly(posPoly,getVar('--pos')); drawPoly(negPoly,getVar('--neg'));}
      // Draw extended line across view for readability
      const [xl,xr] = xWorldSpan(); 
      drawLineSegment(xl, xr);
      drawInterceptMarkersLine();

      const {Apos,Aneg,Anet} = areasLine(xa,xb);
      updateReadouts({
        gtype:'Line',
        segTxt: state.segMode==='full'?'Full View':state.segMode==='axes'?'Clip to Axes':`Domain [${formatNum(state.xMin)}, ${formatNum(state.xMax)}]`,
        Apos,Aneg,Anet, rawUnits, simp,
        intercepts: lineIntercepts()
      });
    } else {
      const {posPoly,negPoly} = areaPolysData();
      if (state.showArea) { // <-- Only draw area if enabled
      drawPoly(posPoly,getVar('--pos')); drawPoly(negPoly,getVar('--neg'));}
      drawPolyline();
      drawInterceptMarkersData();

      const {Apos,Aneg,Anet} = areasData();
      const {xints,yint} = dataIntercepts();
      updateReadouts({
        gtype:'Data',
        segTxt:`Data range`,
        Apos,Aneg,Anet, rawUnits, simp,
        intercepts: { xints: xints.map(x => formatNum(x)), yint: (yint===null? '—' : formatNum(yint)) }
      });
    }
  }

  function updateReadouts({gtype, segTxt, Apos, Aneg, Anet, rawUnits, simp, intercepts}){
    $('#ro_gtype').textContent = gtype;
    ro_seg.textContent = segTxt;
    ro_ap.textContent = formatNum(Apos);
    ro_an.textContent = formatNum(Aneg);
    ro_net.textContent = formatNum(Anet);
    ro_units_raw.textContent = rawUnits;
    ro_units_simple.textContent = simp;
    ro_units_simple2.textContent = simp;
    ro_units_simple3.textContent = simp;
    ro_xu.textContent = state.xLabel;
    ro_yu.textContent = state.yLabel;

    if (state.graphType==='line'){
      ro_m.textContent = formatNum(state.m);
      ro_h.textContent = formatNum(state.h);
    } else {
      ro_m.textContent = '—'; ro_h.textContent = '—';
    }

    if (state.graphType==='line'){
      ro_xint.textContent = Array.isArray(intercepts.xints) ? intercepts.xints.join(', ') : intercepts.xints;
      ro_yint.textContent = intercepts.yint;
    } else {
      const xs = intercepts.xints;
      ro_xint.textContent = xs.length ? (xs.length<=3 ? xs.join(', ') : xs.slice(0,3).join(', ') + ` (+${xs.length-3} more)`) : '—';
      ro_yint.textContent = intercepts.yint;
    }
  }

  // ---------- UI bindings ----------
  function setGraphType(t){
    state.graphType = t;
    if (t==='line'){ modeLineBtn.classList.add('primary'); modeDataBtn.classList.remove('primary'); lineControls.style.display=''; dataControls.style.display='none'; }
    else { modeDataBtn.classList.add('primary'); modeLineBtn.classList.remove('primary'); lineControls.style.display='none'; dataControls.style.display=''; }
    $('#ro_gtype').textContent = t==='line' ? 'Line' : 'Data';
    render();
  }
  modeLineBtn.addEventListener('click', ()=> setGraphType('line'));
  modeDataBtn.addEventListener('click', ()=> setGraphType('data'));

  // Line inputs
  slope.addEventListener('input', ()=>{ state.m=+slope.value; render(); });
  shift.addEventListener('input', ()=>{ state.h=+shift.value; render(); });
  segRadios.forEach(r=>r.addEventListener('change', ()=>{ if(r.checked){ state.segMode=r.value; fixedRow.style.display=(state.segMode==='fixed')?'':'none'; render(); }}));
  xMin.addEventListener('input', ()=>{ state.xMin=+xMin.value; render(); });
  xMax.addEventListener('input', ()=>{ state.xMax=+xMax.value; render(); });

  // Data inputs
  function refreshPointList(){
    if (!state.points.length){ pointList.textContent='—'; return; }
    pointList.textContent = state.points.map((p)=>`(${formatNum(p.x)}, ${formatNum(p.y)})`).join('  ');
  }
  addPoint.addEventListener('click', ()=>{
    const x = +px.value, y=+py.value; if (!isFinite(x)||!isFinite(y)) return;
    state.points.push({x,y}); refreshPointList(); render();
  });
  undoPoint.addEventListener('click', ()=>{ state.points.pop(); refreshPointList(); render(); });
  clearPoints.addEventListener('click', ()=>{ state.points.length=0; refreshPointList(); render(); });
  importCSV.addEventListener('click', ()=>{
    const text = csvBox.value.trim(); if (!text) return;
    const lines = text.split(/\r?\n/);
    for (const ln of lines){
      const m = ln.trim().match(/^\s*([-+]?[\d.]+(?:e[-+]?\d+)?)\s*[,;\s]\s*([-+]?[\d.]+(?:e[-+]?\d+)?)\s*$/i);
      if (m){ state.points.push({x:+m[1], y:+m[2]}); }
    }
    refreshPointList(); render();
  });
  fitDataBtn.addEventListener('click', ()=>{
    if (state.points.length<2) return;
    const xs=state.points.map(p=>p.x), ys=state.points.map(p=>p.y);
    const a=Math.min(...xs), b=Math.max(...xs), ymin=Math.min(0,...ys), ymax=Math.max(0,...ys);
    const rect = canvas.getBoundingClientRect(), pad=0.12;
    state.scaleX = (rect.width*(1-2*pad))/Math.max(1e-9,b-a);
    state.scaleY = (rect.height*(1-2*pad))/Math.max(1e-9,(ymax-ymin)||1);
    const cxW=(a+b)/2, cyW=(ymin+ymax)/2;
    state.tx = rect.width/2 - cxW*state.scaleX;
    state.ty = rect.height/2 + cyW*state.scaleY;
    customAxes.checked = false; state.customAxes = false; customAxesSection.style.display='none';
    render();
  });

  // Custom Axes
  customAxes.addEventListener('change', ()=>{
    state.customAxes = customAxes.checked;
    customAxesSection.style.display = state.customAxes ? '' : 'none';
    render();
  });
  xAxisMin.addEventListener('input', ()=>{ state.xAxisMin=+xAxisMin.value; render(); });
  xAxisMax.addEventListener('input', ()=>{ state.xAxisMax=+xAxisMax.value; render(); });
  xAxisInc.addEventListener('input', ()=>{ state.xAxisInc=+xAxisInc.value; render(); });
  xLabelNth.addEventListener('input', ()=>{ state.xLabelNth=+xLabelNth.value; render(); });
  yAxisMin.addEventListener('input', ()=>{ state.yAxisMin=+yAxisMin.value; render(); });
  yAxisMax.addEventListener('input', ()=>{ state.yAxisMax=+yAxisMax.value; render(); });
  yAxisInc.addEventListener('input', ()=>{ state.yAxisInc=+yAxisInc.value; render(); });
  yLabelNth.addEventListener('input', ()=>{ state.yLabelNth=+yLabelNth.value; render(); });
  xAxisLabel.addEventListener('input', ()=>{ state.xLabel=xAxisLabel.value.trim()||'x'; render(); });
  yAxisLabel.addEventListener('input', ()=>{ state.yLabel=yAxisLabel.value.trim()||'y'; render(); });
  xUseTickIndex.addEventListener('change', ()=>{ state.xUseTickIndex = xUseTickIndex.checked; render(); });
  xTickIdxMin.addEventListener('input', ()=>{ state.xTickIdxMin = +xTickIdxMin.value; render(); });
  xTickIdxMax.addEventListener('input', ()=>{ state.xTickIdxMax = +xTickIdxMax.value; render(); });
  yUseTickIndex.addEventListener('change', ()=>{ state.yUseTickIndex = yUseTickIndex.checked; render(); });
  yTickIdxMin.addEventListener('input', ()=>{ state.yTickIdxMin = +yTickIdxMin.value; render(); });
  yTickIdxMax.addEventListener('input', ()=>{ state.yTickIdxMax = +yTickIdxMax.value; render(); });
  viewPadPct.addEventListener('input', ()=>{
    const v = Math.max(0, Math.min(40, +viewPadPct.value || 0));
    state.viewPad = v / 100; render();
  });

  crosshair.addEventListener('change', ()=>{ state.showCrosshair=crosshair.checked; ro_cursor.style.display = state.showCrosshair ? '' : 'none'; render(); });
  markers.addEventListener('change', ()=>{ state.showMarkers=markers.checked; render(); });
  resetLabelPos.addEventListener('click', ()=>{
    state.labelOffsetX = {dx:0,dy:0};
    state.labelOffsetY = {dx:0,dy:0};
    render();
  });

  showArea.addEventListener('change', ()=>{
  state.showArea = showArea.checked;
  render();
});

  // ---------- Mouse interactions (pan/zoom + label dragging) ----------
  function withinRect(r, x, y){ return x>=r.x && x<=r.x+r.w && y>=r.y && y<=r.y+r.h; }

  function zoomAt(xs,ys,f){
    state.customAxes = false; customAxes.checked = false; customAxesSection.style.display = 'none';
    const [wx,wy]=toWorld(xs,ys);
    state.scaleX*=f; state.scaleY*=f;
    state.tx=xs-wx*state.scaleX;
    state.ty=ys+wy*state.scaleY;
    render();
  }
  fitAll.addEventListener('click', ()=>{
    // Fit to all data, line segment, or custom axes
    const rect = canvas.getBoundingClientRect();
    let pad = Math.max(0, Math.min(0.40, state.viewPad ?? 0.08));
    if (state.customAxes) {
      // Use custom axes domain
      const dom = computeCustomDomainAndView();
      // Camera from padded view box
      state.scaleX = rect.width  / Math.max(1e-9, dom.xMaxView - dom.xMinView);
      state.scaleY = rect.height / Math.max(1e-9, dom.yMaxView - dom.yMinView);
      state.tx = -dom.xMinView * state.scaleX;
      state.ty = rect.height + dom.yMinView * state.scaleY;
    } else if (state.graphType === 'data' && state.points.length > 0) {
      // Fit all data points
      const xs = state.points.map(p => p.x), ys = state.points.map(p => p.y);
      const xMin = Math.min(...xs), xMax = Math.max(...xs);
      const yMin = Math.min(...ys), yMax = Math.max(...ys);
      const xRange = Math.max(1e-9, xMax - xMin);
      const yRange = Math.max(1e-9, yMax - yMin);
      const xMinView = xMin - pad * xRange;
      const xMaxView = xMax + pad * xRange;
      const yMinView = yMin - pad * yRange;
      const yMaxView = yMax + pad * yRange;
      state.scaleX = rect.width / Math.max(1e-9, xMaxView - xMinView);
      state.scaleY = rect.height / Math.max(1e-9, yMaxView - yMinView);
      state.tx = -xMinView * state.scaleX;
      state.ty = rect.height + yMinView * state.scaleY;
    } else {
      // Line mode: fit segment or domain
      let xMin, xMax, yMin, yMax;
      if (state.segMode === 'fixed') {
        xMin = state.xMin;
        xMax = state.xMax;
      } else {
        // Use a reasonable default domain for full/axes mode
        xMin = -12;
        xMax = 12;
      }
      yMin = Math.min(yLine(xMin), yLine(xMax));
      yMax = Math.max(yLine(xMin), yLine(xMax));
      const xRange = Math.max(1e-9, xMax - xMin);
      const yRange = Math.max(1e-9, yMax - yMin);
      const xMinView = xMin - pad * xRange;
      const xMaxView = xMax + pad * xRange;
      const yMinView = yMin - pad * yRange;
      const yMaxView = yMax + pad * yRange;
      state.scaleX = rect.width / Math.max(1e-9, xMaxView - xMinView);
      state.scaleY = rect.height / Math.max(1e-9, yMaxView - yMinView);
      state.tx = -xMinView * state.scaleX;
      state.ty = rect.height + yMinView * state.scaleY;
    }
    customAxes.checked = state.customAxes; customAxesSection.style.display = state.customAxes ? '' : 'none';
    render();
  });
  zoomIn.addEventListener('click', ()=> zoomAt(canvas.width/2, canvas.height/2, 1.2));
  zoomOut.addEventListener('click', ()=> zoomAt(canvas.width/2, canvas.height/2, 1/1.2));
  resetView.addEventListener('click', ()=>{
    const rect = canvas.getBoundingClientRect();
    state.scaleX = rect.width/24;
    state.scaleY = rect.height/16;
    state.tx=rect.width/2; state.ty=rect.height/2;
    customAxes.checked = false; state.customAxes = false; customAxesSection.style.display = 'none';
    render();
  });
  exportBtn.addEventListener('click', ()=>{
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const rect = canvas.getBoundingClientRect();
    const backup = {w:canvas.width,h:canvas.height, tx:state.tx, ty:state.ty};
    canvas.width = Math.round(rect.width*dpr); canvas.height = Math.round(rect.height*dpr);
    ctx.setTransform(dpr,0,0,dpr,0,0);
    state.tx *= dpr; state.ty *= dpr;
    render();
    const url = canvas.toDataURL('image/png');
    canvas.width = backup.w; canvas.height = backup.h; fitCanvas();
    state.tx = backup.tx; state.ty = backup.ty; render();
    const a=document.createElement('a'); a.href=url; a.download='graph.png'; a.click();
  });

  let isPanning=false, last={x:0,y:0}, spaceDown=false, draggingH=false;
  let draggingLabel=null; // 'x' | 'y' | null
  let dragStart={x:0,y:0}, startOffset={dx:0,dy:0};

  window.addEventListener('keydown', e=>{ if (e.code==='Space'){ spaceDown=true; }});
  window.addEventListener('keyup', e=>{ if (e.code==='Space'){ spaceDown=false; }});

  canvas.addEventListener('mousedown', e=>{
    const rect = canvas.getBoundingClientRect(), x=e.clientX-rect.left, y=e.clientY-rect.top;

    // Drag axis labels if clicked
    if (withinRect(labelRects.x, x, y)){
      draggingLabel='x'; dragStart={x:e.clientX,y:e.clientY}; startOffset={...state.labelOffsetX}; e.preventDefault(); return;
    }
    if (withinRect(labelRects.y, x, y)){
      draggingLabel='y'; dragStart={x:e.clientX,y:e.clientY}; startOffset={...state.labelOffsetY}; e.preventDefault(); return;
    }

    if (spaceDown || e.button===1){
      isPanning=true; last.x=e.clientX; last.y=e.clientY; e.preventDefault();
      customAxes.checked = false; state.customAxes = false; customAxesSection.style.display = 'none';
    }
    else if (state.graphType==='line'){
      const [wx,wy]=toWorld(x,y);
      if (Math.abs(wy - yLine(wx))<=0.4) draggingH=true;
    }
  });

  window.addEventListener('mouseup', ()=>{ isPanning=false; draggingH=false; draggingLabel=null; canvas.style.cursor='default'; });

  canvas.addEventListener('mousemove', e=>{
    const rect = canvas.getBoundingClientRect();
    const xs=e.clientX-rect.left, ys=e.clientY-rect.top;

    // hover cursor for labels
    if (!isPanning && !draggingH && !draggingLabel){
      if (withinRect(labelRects.x, xs, ys) || withinRect(labelRects.y, xs, ys)) canvas.style.cursor='move';
      else canvas.style.cursor='default';
    }

    if (draggingLabel==='x'){
      state.labelOffsetX = { dx: startOffset.dx + (e.clientX - dragStart.x), dy: startOffset.dy + (e.clientY - dragStart.y) };
      render(); return;
    }
    if (draggingLabel==='y'){
      state.labelOffsetY = { dx: startOffset.dx + (e.clientX - dragStart.x), dy: startOffset.dy + (e.clientY - dragStart.y) };
      render(); return;
    }

    if (state.showCrosshair){ const [wx,wy]=toWorld(xs,ys); $('#cx').textContent=formatNum(wx); $('#cy').textContent=formatNum(wy); }
    if (isPanning){ state.tx += (e.clientX-last.x); state.ty += (e.clientY-last.y); last.x=e.clientX; last.y=e.clientY; render(); return; }
    if (draggingH){
      const [wx,wy]=toWorld(xs,ys);
      if (Math.abs(state.m)>1e-9) state.h = wx - wy/state.m; else state.h = wx;
      shift.value = formatNum(state.h); render(); return;
    }
  });

  // Double-click to reset label you clicked
  canvas.addEventListener('dblclick', e=>{
    const rect = canvas.getBoundingClientRect(), x=e.clientX-rect.left, y=e.clientY-rect.top;
    if (withinRect(labelRects.x, x, y)){ state.labelOffsetX={dx:0,dy:0}; render(); }
    else if (withinRect(labelRects.y, x, y)){ state.labelOffsetY={dx:0,dy:0}; render(); }
  });

  canvas.addEventListener('wheel', e=>{
    e.preventDefault();
    const rect = canvas.getBoundingClientRect(), xs=e.clientX-rect.left, ys=e.clientY-rect.top;
    const zoom = Math.pow(1.0015, -e.deltaY); zoomAt(xs,ys,zoom);
  }, {passive:false});

  // Init
  window.addEventListener('resize', ()=>{ fitCanvas(); render(); });
  fitCanvas(); initCamera(); render();

  /* ========= NEXT: Derivative & Integral series (for tri-graph) =========
     These helpers compute new series from the current polyline (Data mode).
     - derivativeSeries(): piecewise-slope using central differences (endpoints: one-sided)
     - integralSeries(): cumulative trapezoid (signed). Set absoluteAreas=true for unsigned accumulation.
  */
  function derivativeSeries(points){
    if (!points || points.length<2) return [];
    // ensure increasing x (keep original order for now; skip verticals)
    const P = points.filter((_,i)=> i===0 || points[i].x!==points[i-1].x);
    const n = P.length;
    const D = [];
    for (let i=0;i<n;i++){
      let s;
      if (i===0){
        const dx=P[1].x-P[0].x; s = dx!==0 ? (P[1].y-P[0].y)/dx : 0;
      } else if (i===n-1){
        const dx=P[i].x-P[i-1].x; s = dx!==0 ? (P[i].y-P[i-1].y)/dx : 0;
      } else {
        const dx=P[i+1].x-P[i-1].x; s = dx!==0 ? (P[i+1].y-P[i-1].y)/dx : 0;
      }
      D.push({x:P[i].x, y:s});
    }
    return D;
  }
  function integralSeries(points, absoluteAreas=false){
    if (!points || points.length<2) return [];
    const P = points.slice();
    const A = [{x:P[0].x, y:0}];
    let acc = 0;
    for (let i=0;i<P.length-1;i++){
      const dx = (P[i+1].x - P[i].x);
      if (dx===0) continue;
      const trap = (P[i].y + P[i+1].y)/2 * dx;
      acc += absoluteAreas ? Math.abs(trap) : trap;
      A.push({x:P[i+1].x, y:acc});
    }
    return A;
  }
  // (When we build the tri-graph, we’ll render these on linked canvases with shared x-range.)
})();