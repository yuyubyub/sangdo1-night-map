// 암부(가로등 불빛이 닿지 않는 도로 구간) 계산 로직. 지도와 무관한 순수 함수라서 따로 테스트할 수 있어요.
(function (root) {
  // 위경도 -> 미터 단위 평면좌표 (기준점 근처에서만 쓰는 근사)
  function makeProj(lat0, lon0) {
    const ky = 111320;
    const kx = 111320 * Math.cos((lat0 * Math.PI) / 180);
    return { lat0, lon0, kx, ky, toXY: (lat, lon) => [(lon - lon0) * kx, (lat - lat0) * ky] };
  }

  // 볼록 껍질 (monotone chain): 가로등이 있는 범위를 '조사 구역'으로 삼기 위해 사용
  function hull(points) {
    const p = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    if (p.length < 3) return p;
    const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = [];
    for (const q of p) {
      while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop();
      lo.push(q);
    }
    const up = [];
    for (let i = p.length - 1; i >= 0; i--) {
      const q = p[i];
      while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop();
      up.push(q);
    }
    lo.pop();
    up.pop();
    return lo.concat(up);
  }

  // 다각형을 중심 기준으로 d미터만큼 바깥으로 넓힘 (경계 도로가 잘리는 것 완화)
  function inflate(poly, d) {
    const cx = poly.reduce((s, q) => s + q[0], 0) / poly.length;
    const cy = poly.reduce((s, q) => s + q[1], 0) / poly.length;
    return poly.map(([x, y]) => {
      const dx = x - cx, dy = y - cy, len = Math.hypot(dx, dy) || 1;
      return [x + (dx / len) * d, y + (dy / len) * d];
    });
  }

  function inPoly(pt, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i], [xj, yj] = poly[j];
      if (yi > pt[1] !== yj > pt[1] && pt[0] < ((xj - xi) * (pt[1] - yi)) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }

  // 도로(way) 하나를 step미터 간격의 점들로 쪼갬. ways: [{id, geometry:[{lat,lon},...]}]
  function sampleWays(ways, proj, step) {
    const lines = [];
    for (const w of ways) {
      const g = w.geometry || [];
      if (g.length < 2) continue;
      const s = [];
      for (let i = 0; i < g.length - 1; i++) {
        const a = proj.toXY(g[i].lat, g[i].lon), b = proj.toXY(g[i + 1].lat, g[i + 1].lon);
        const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
        const n = Math.max(1, Math.ceil(L / step));
        for (let k = 0; k < n; k++) {
          const t = k / n;
          s.push({
            x: a[0] + (b[0] - a[0]) * t,
            y: a[1] + (b[1] - a[1]) * t,
            lat: g[i].lat + (g[i + 1].lat - g[i].lat) * t,
            lon: g[i].lon + (g[i + 1].lon - g[i].lon) * t,
          });
        }
      }
      const last = g[g.length - 1], lp = proj.toXY(last.lat, last.lon);
      s.push({ x: lp[0], y: lp[1], lat: last.lat, lon: last.lon });
      lines.push({ id: w.id, tags: w.tags || {}, samples: s });
    }
    return lines;
  }

  // 각 점에서 가장 가까운 가로등까지의 거리(d)와 조사 구역 안/밖(inScope)을 기록
  function annotate(lines, lightsXY, poly) {
    for (const ln of lines) {
      for (const p of ln.samples) {
        let best = Infinity;
        for (const l of lightsXY) {
          const d = Math.hypot(p.x - l[0], p.y - l[1]);
          if (d < best) best = d;
        }
        p.d = best;
        p.inScope = inPoly([p.x, p.y], poly);
      }
    }
  }

  // 반경 R보다 멀리 떨어진 연속 구간(암부)을 찾음
  function findDark(lines, R, minLen) {
    const segments = [];
    let total = 0, dark = 0;
    for (const ln of lines) {
      const s = ln.samples;
      let run = null;
      const flush = () => {
        if (run && run.len >= minLen && run.pts.length >= 2) {
          segments.push({ wayId: ln.id, name: ln.tags.name || "", tags: ln.tags, points: run.pts, length: run.len, maxD: run.maxD });
          dark += run.len;
        }
        run = null;
      };
      for (let i = 0; i < s.length; i++) {
        const p = s[i];
        const seg = i > 0 ? Math.hypot(p.x - s[i - 1].x, p.y - s[i - 1].y) : 0;
        if (p.inScope) total += seg;
        if (p.inScope && p.d > R) {
          if (!run) run = { pts: [], len: 0, maxD: 0 };
          else run.len += seg;
          run.pts.push({ lat: p.lat, lon: p.lon });
          if (p.d > run.maxD) run.maxD = p.d;
        } else flush();
      }
      flush();
    }
    return { segments, totalLen: total, darkLen: dark };
  }

  const api = { makeProj, hull, inflate, inPoly, sampleWays, annotate, findDark };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Analysis = api;
})(typeof window !== "undefined" ? window : globalThis);