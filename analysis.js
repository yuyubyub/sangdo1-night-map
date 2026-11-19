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

  // 조사 구역(poly) 안/밖만 표시
  function markScope(lines, poly) {
    for (const ln of lines) for (const p of ln.samples) p.inScope = inPoly([p.x, p.y], poly);
  }

  // 각 도로 점의 지면 수평면 조도(lx) = 모든 등의 합.
  // src: [{x, y, H, I}]  H = 설치 높이(m), I = 광도(cd) = 광속(lm) / 2π (하반구 균일 배광)
  // 점광원이 높이 H에 있을 때 수평 거리 r 지점의 조도: E = I·H / (r² + H²)^(3/2)
  function computeE(lines, src, cut) {
    const c2 = (cut || 60) * (cut || 60);
    for (const ln of lines) {
      for (const p of ln.samples) {
        if (!p.inScope) { p.E = Infinity; continue; }
        let E = 0;
        for (const s of src) {
          const dx = p.x - s.x, dy = p.y - s.y, r2 = dx * dx + dy * dy;
          if (r2 > c2) continue;
          const d2 = r2 + s.H * s.H;
          E += (s.I * s.H) / (d2 * Math.sqrt(d2));
        }
        p.E = E;
      }
    }
  }

  // 기준 조도 thr(lx) 미만인 연속 구간(minLen m 이상)을 암부로 추출
  function findDarkE(lines, thr, minLen) {
    const segments = [];
    let total = 0, dark = 0;
    for (const ln of lines) {
      const s = ln.samples;
      let run = null;
      const flush = () => {
        if (run && run.len >= minLen && run.pts.length >= 2) {
          segments.push({ wayId: ln.id, name: ln.tags.name || "", tags: ln.tags, points: run.pts, length: run.len, minE: run.minE });
          dark += run.len;
        }
        run = null;
      };
      for (let i = 0; i < s.length; i++) {
        const p = s[i];
        const seg = i > 0 ? Math.hypot(p.x - s[i - 1].x, p.y - s[i - 1].y) : 0;
        if (p.inScope) total += seg;
        if (p.inScope && p.E < thr) {
          if (!run) run = { pts: [], len: 0, minE: Infinity };
          else run.len += seg;
          run.pts.push({ lat: p.lat, lon: p.lon });
          if (p.E < run.minE) run.minE = p.E;
        } else flush();
      }
      flush();
    }
    return { segments, totalLen: total, darkLen: dark };
  }

  // ───────── 구간(등과 등 사이) 평균 조도 · 균제도 · 등급 ─────────
  // 폭 방향 5개 지점(폭 6 m를 1.2 m씩 5등분한 각 구간의 중앙). 표 8의 폭 6 m 영역을 근사한다.
  const OFFSETS = [-2.4, -1.2, 0, 1.2, 2.4];

  function cumulate(ln) {
    if (ln.cum) return;
    const s = ln.samples;
    ln.cum = new Array(s.length);
    let acc = 0;
    for (let i = 0; i < s.length; i++) {
      if (i > 0) acc += Math.hypot(s[i].x - s[i - 1].x, s[i].y - s[i - 1].y);
      ln.cum[i] = acc;
    }
  }

  // 도로 위 각 점에서, 도로와 직각 방향 오프셋 지점들의 지면 조도(lx)를 구해 p.El 에 저장.
  // src: [{x, y, H, I}], cut: 이 거리(m)보다 먼 등은 계산에서 제외
  function computeStrip(lines, src, cut, offsets) {
    const offs = offsets || OFFSETS;
    const c = cut || 60, c2 = c * c;
    const maxOff = Math.max(...offs.map(Math.abs));
    const near = (c + maxOff) * (c + maxOff);
    for (const ln of lines) {
      cumulate(ln);
      const s = ln.samples, n = s.length;
      for (let i = 0; i < n; i++) {
        const a = s[Math.max(0, i - 1)], b = s[Math.min(n - 1, i + 1)];
        let tx = b.x - a.x, ty = b.y - a.y;
        const L = Math.hypot(tx, ty) || 1;
        tx /= L; ty /= L;
        const nx = -ty, ny = tx, p = s[i];
        const cand = [];
        for (const q of src) {
          const dx = p.x - q.x, dy = p.y - q.y;
          if (dx * dx + dy * dy <= near) cand.push(q);
        }
        p.El = offs.map((o) => {
          const px = p.x + nx * o, py = p.y + ny * o;
          let E = 0;
          for (const q of cand) {
            const dx = px - q.x, dy = py - q.y, r2 = dx * dx + dy * dy;
            if (r2 > c2) continue;
            const d2 = r2 + q.H * q.H;
            E += (q.I * q.H) / (d2 * Math.sqrt(d2));
          }
          return E;
        });
      }
    }
  }

  // 각 도로를 '등과 등 사이' 구간(셀)으로 나눈다.
  // pts: 등 위치 [[x,y],...]. 도로에서 maxLat(m) 이내인 등을 그 도로 위의 경계점으로 본다.
  // minCell(m)보다 짧은 구간은 이웃 구간과 합친다.
  function buildCells(lines, pts, maxLat, minCell) {
    const ml = (maxLat || 25) * (maxLat || 25), mc = minCell || 6;
    for (const ln of lines) {
      cumulate(ln);
      const s = ln.samples, n = s.length;
      const total = ln.cum[n - 1];
      const cuts = [];
      for (const q of pts) {
        let best = Infinity, bi = -1;
        for (let i = 0; i < n; i++) {
          const dx = s[i].x - q[0], dy = s[i].y - q[1], d = dx * dx + dy * dy;
          if (d < best) { best = d; bi = i; }
        }
        if (best <= ml) cuts.push(ln.cum[bi]);
      }
      cuts.sort((a, b) => a - b);
      const bounds = [0];
      for (const c of cuts) {
        if (c - bounds[bounds.length - 1] >= mc && total - c >= mc) bounds.push(c);
      }
      bounds.push(total);
      ln.cells = [];
      for (let k = 0; k < bounds.length - 1; k++) ln.cells.push({ a: bounds[k], b: bounds[k + 1] });
    }
  }

  // 등급: 4 취약(평균 e3 미만 또는 균제도 u 미만) / 3 미달 / 2 충족 / 1 양호
  function gradeOf(avg, U, T) {
    if (!(avg >= 0)) return 0;
    if (avg < T.e3 || U < T.u) return 4;
    if (avg < T.e2) return 3;
    if (avg < T.e1) return 2;
    return 1;
  }

  // 각 구간(셀)의 평균 조도(Ē)·최저 조도·균제도(최저÷평균)·등급을 계산해 셀과 점(p.grade)에 기록.
  // 점은 [a, b) 구간에 속한다(마지막 셀만 끝점 포함). 평균은 구간 전체 점의 폭 방향 샘플 평균.
  function evalCells(lines, th) {
    const T = Object.assign({ e1: 5, e2: 3, e3: 2, u: 0.15 }, th);
    for (const ln of lines) {
      const s = ln.samples, last = ln.cells.length - 1;
      ln.cells.forEach((cell, k) => {
        let sum = 0, cnt = 0, mn = Infinity;
        cell.idx = [];
        for (let i = 0; i < s.length; i++) {
          const c = ln.cum[i];
          if (c >= cell.a && (c < cell.b || (k === last && c <= cell.b))) {
            cell.idx.push(i);
            for (const e of s[i].El) { sum += e; cnt++; if (e < mn) mn = e; }
          }
        }
        cell.avg = cnt ? sum / cnt : 0;
        cell.min = cnt ? mn : 0;
        cell.U = cell.avg > 0 ? cell.min / cell.avg : 0;
        cell.grade = gradeOf(cell.avg, cell.U, T);
        cell.len = cell.b - cell.a;
        for (const i of cell.idx) { s[i].grade = cell.grade; s[i].cell = cell; }
      });
    }
    return T;
  }

  // 조사 구역 안 점들의 길이를 등급별로 합산하고, 3·4등급 구간(minLen m 이상)을 연속 구간으로 묶어 반환
  function summarize(lines, minLen) {
    const len = { 1: 0, 2: 0, 3: 0, 4: 0 };
    let total = 0;
    const segments = [];
    for (const ln of lines) {
      const s = ln.samples;
      for (let i = 0; i < s.length; i++) {
        if (!s[i].inScope || !s[i].grade) continue;
        const seg = i > 0 ? Math.hypot(s[i].x - s[i - 1].x, s[i].y - s[i - 1].y) : 0;
        len[s[i].grade] += seg;
        total += seg;
      }
      let cur = null;
      const flush = () => {
        if (cur && cur.points.length >= 2) {
          let L = 0;
          for (let j = 1; j < cur.points.length; j++) L += Math.hypot(cur.points[j].x - cur.points[j - 1].x, cur.points[j].y - cur.points[j - 1].y);
          if (L >= minLen) segments.push({ wayId: ln.id, name: ln.tags.name || "", tags: ln.tags, points: cur.points, length: L, grade: cur.grade, minAvg: cur.minAvg, minU: cur.minU, maxSpan: cur.maxSpan });
        }
        cur = null;
      };
      for (const cell of ln.cells) {
        const pts = cell.idx.filter((i) => s[i].inScope).map((i) => ({ lat: s[i].lat, lon: s[i].lon, x: s[i].x, y: s[i].y }));
        if (cell.grade < 3 || pts.length < 1) { flush(); continue; }
        if (cur && cur.grade !== cell.grade) flush();
        if (!cur) cur = { grade: cell.grade, points: [], minAvg: Infinity, minU: Infinity, maxSpan: 0 };
        cur.points.push(...pts);
        cur.minAvg = Math.min(cur.minAvg, cell.avg);
        cur.minU = Math.min(cur.minU, cell.U);
        cur.maxSpan = Math.max(cur.maxSpan, cell.len);
      }
      flush();
    }
    return { len, total, segments };
  }

  const api = { makeProj, hull, inflate, inPoly, sampleWays, annotate, findDark, markScope, computeE, findDarkE,
    OFFSETS, computeStrip, buildCells, gradeOf, evalCells, summarize };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Analysis = api;
})(typeof window !== "undefined" ? window : globalThis);