"use strict";
// Claude Code 作業履歴ビューア(フロントエンド)。外部ライブラリは使わない。

// ---------- 定数 ----------
const STATUS = {
  running: { label: "作業中", order: 0 },
  waiting: { label: "入力待ち", order: 1 },
  interrupted: { label: "中断", order: 2 },
  error: { label: "エラー", order: 3 },
  ended: { label: "終了", order: 4 },
};
const LAST_KIND = {
  prompt: "ユーザーの発言を受信(応答待ち)",
  tool_result: "ツールの実行結果を受信",
  assistant_tool_use: "ツールを実行中",
  assistant_end: "応答を完了",
  interrupt: "ユーザーが中断",
  api_error: "APIエラーで停止",
};
const WORK_TYPES = ["実装", "調査", "実行・検証", "会話"];
const SERIES = ["--s1", "--s2", "--s3", "--s4", "--s5", "--s6", "--s7", "--s8"];
const COLOR_BY = {
  workType: "作業種別",
  project: "プロジェクト",
  status: "状態",
  branch: "ブランチ",
  session: "セッション",
};
const ISSUE_KIND = { tool_error: "ツールエラー", interrupt: "中断", api_error: "APIエラー" };
const DAY_MS = 86400000;
const WEEKDAYS = ["日", "月", "火", "水", "木", "金", "土"];

// ---------- 状態 ----------
const state = {
  sessions: new Map(),
  view: "dashboard",
  sessionMode: "calendar",
  colorBy: "workType",
  statusFilter: new Set(),
  issueFilter: new Set(),
  taskScope: "all",
  sort: "recent",
  search: "",
  weekStart: startOfWeek(new Date()),
  hourPx: 40,
  selected: null,
  logOpen: false,
  serverOffset: 0,
  recentlyUpdated: new Set(),
  colorSlots: {},
};

// 表示設定はブラウザに保存(失敗しても動くようにする)
const PREF_KEYS = ["view", "sessionMode", "colorBy", "hourPx", "taskScope", "sort"];
try {
  const saved = JSON.parse(localStorage.getItem("cchv-prefs") || "{}");
  for (const k of PREF_KEYS) if (saved[k] !== undefined) state[k] = saved[k];
} catch (_) { /* 保存領域が使えなくても既定値で動く */ }
function savePrefs() {
  try {
    localStorage.setItem("cchv-prefs", JSON.stringify(Object.fromEntries(PREF_KEYS.map((k) => [k, state[k]]))));
  } catch (_) { /* 無視 */ }
}

// ---------- 小道具 ----------
function $(sel, root = document) { return root.querySelector(sel); }
function esc(s) {
  return String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
function now() { return Date.now() / 1000 + state.serverOffset; }
function startOfWeek(d) {
  const x = new Date(d.getFullYear(), d.getMonth(), d.getDate());
  const wd = (x.getDay() + 6) % 7; // 月曜始まり
  x.setDate(x.getDate() - wd);
  return x;
}
function addDays(d, n) { const x = new Date(d); x.setDate(x.getDate() + n); return x; }
function pad(n) { return String(n).padStart(2, "0"); }
function fmtTime(ts) { if (!ts) return "-"; const d = new Date(ts * 1000); return `${pad(d.getHours())}:${pad(d.getMinutes())}`; }
function fmtDate(ts) { if (!ts) return "-"; const d = new Date(ts * 1000); return `${d.getMonth() + 1}/${d.getDate()}(${WEEKDAYS[d.getDay()]})`; }
function fmtDateTime(ts) { return ts ? `${fmtDate(ts)} ${fmtTime(ts)}` : "-"; }
function fmtDur(sec) {
  sec = Math.round(sec || 0);
  if (sec < 60) return `${sec}秒`;
  const m = Math.round(sec / 60);
  if (m < 60) return `${m}分`;
  return `${Math.floor(m / 60)}時間${m % 60 ? (m % 60) + "分" : ""}`;
}
function fmtAgo(ts) {
  if (!ts) return "-";
  const s = now() - ts;
  if (s < 60) return "たった今";
  if (s < 3600) return `${Math.floor(s / 60)}分前`;
  if (s < 86400) return `${Math.floor(s / 3600)}時間前`;
  return `${Math.floor(s / 86400)}日前`;
}
function fmtTok(n) {
  n = n || 0;
  if (n >= 1e6) return (n / 1e6).toFixed(1) + "M";
  if (n >= 1e3) return (n / 1e3).toFixed(1) + "k";
  return String(n);
}
function sum(arr, f = (x) => x) { return arr.reduce((a, b) => a + f(b), 0); }
function statusBadge(s) { return `<span class="status ${s.status}">${STATUS[s.status].label}${s.statusSource === "transcript" && s.status === "running" ? "(推定)" : ""}</span>`; }
function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

// ---------- 色分け ----------
// 色は「対象の実体」に固定する(絞り込みで色が入れ替わらないよう、開始順で割り当てて保持する)
function colorKey(s, by = state.colorBy) {
  switch (by) {
    case "workType": return s.workType;
    case "project": return s.projectName || "(不明)";
    case "status": return s.status;
    case "branch": return s.branch || "(なし)";
    default: return s.id;
  }
}
function computeColorSlots() {
  const slots = {};
  const byStart = [...state.sessions.values()].sort((a, b) => (a.start || 0) - (b.start || 0));
  for (const by of ["project", "branch", "session"]) {
    const map = new Map();
    for (const s of byStart) {
      const k = colorKey(s, by);
      if (!map.has(k)) map.set(k, map.size < SERIES.length ? `var(${SERIES[map.size]})` : "var(--other)");
    }
    slots[by] = map;
  }
  slots.workType = new Map(WORK_TYPES.map((w, i) => [w, `var(${SERIES[i]})`]));
  slots.status = new Map([
    ["running", "var(--good)"], ["waiting", "var(--warning)"], ["interrupted", "var(--serious)"],
    ["error", "var(--critical)"], ["ended", "var(--other)"],
  ]);
  state.colorSlots = slots;
}
function colorOf(s, by = state.colorBy) {
  return state.colorSlots[by]?.get(colorKey(s, by)) || "var(--other)";
}
function legendHtml(list) {
  const by = state.colorBy;
  const counts = new Map();
  for (const s of list) counts.set(colorKey(s), (counts.get(colorKey(s)) || 0) + 1);
  let keys = [...state.colorSlots[by].keys()].filter((k) => counts.has(k));
  if (by === "session" || keys.length > 12) {
    const shown = keys.filter((k) => state.colorSlots[by].get(k) !== "var(--other)").slice(0, 8);
    const rest = sum(keys.filter((k) => !shown.includes(k)), (k) => counts.get(k));
    keys = shown;
    const items = keys.map((k) => {
      const s = list.find((x) => colorKey(x) === k);
      const label = by === "session" ? (s?.title || k).slice(0, 18) : k;
      return `<span><i class="swatch" style="--c:${state.colorSlots[by].get(k)}"></i>${esc(label)}</span>`;
    });
    if (rest) items.push(`<span><i class="swatch" style="--c:var(--other)"></i>その他 ${rest}</span>`);
    return `<div class="legend">${items.join("")}</div>`;
  }
  return `<div class="legend">${keys.map((k) => {
    const label = by === "status" ? STATUS[k].label : k;
    return `<span><i class="swatch" style="--c:${state.colorSlots[by].get(k)}"></i>${esc(label)} <span class="num muted">${counts.get(k)}</span></span>`;
  }).join("")}${by === "workType" ? `<span class="muted">※作業種別はツール使用回数からの推定</span>` : ""}</div>`;
}

// ---------- 絞り込み ----------
function matchesSearch(s) {
  const q = state.search.trim().toLowerCase();
  if (!q) return true;
  const hay = [s.title, s.firstPrompt, s.project, s.branch, s.id, ...(s.prompts || []).map((p) => p.text)].join("\n").toLowerCase();
  return q.split(/\s+/).every((w) => hay.includes(w));
}
function visibleSessions({ useStatus = true } = {}) {
  return [...state.sessions.values()].filter((s) => matchesSearch(s) && (!useStatus || !state.statusFilter.size || state.statusFilter.has(s.status)));
}
function effectiveEnd(s) {
  // 作業中のセッションは現在時刻まで伸ばして表示する
  return s.status === "running" ? Math.max(s.end || 0, now()) : s.end;
}

// ---------- データ取得・リアルタイム更新 ----------
async function loadState() {
  const res = await fetch("/api/state");
  const data = await res.json();
  state.serverOffset = data.serverTime - Date.now() / 1000;
  state.sessions = new Map(data.sessions.map((s) => [s.id, s]));
  computeColorSlots();
  render();
}

function connectEvents() {
  const live = $("#live");
  const es = new EventSource("/api/events");
  const setLive = (on, text) => {
    live.className = "live " + (on ? "on" : "off");
    live.querySelector("span").textContent = text;
  };
  es.addEventListener("hello", () => setLive(true, "リアルタイム更新中"));
  es.addEventListener("update", (ev) => {
    const data = JSON.parse(ev.data);
    state.serverOffset = data.serverTime - Date.now() / 1000;
    for (const s of data.sessions) {
      state.sessions.set(s.id, s);
      state.recentlyUpdated.add(s.id);
    }
    computeColorSlots();
    setLive(true, `更新 ${fmtTime(data.serverTime)}`);
    render({ keepScroll: true });
    if (state.logOpen && data.sessions.some((s) => s.id === state.selected)) loadLog(state.selected);
    setTimeout(() => { for (const s of data.sessions) state.recentlyUpdated.delete(s.id); }, 1500);
  });
  es.onerror = () => setLive(false, "再接続中…");
}

// ---------- 描画 ----------
function render({ keepScroll = false } = {}) {
  const main = $("#main");
  const scroll = main.scrollTop;
  document.querySelectorAll("#tabs button").forEach((b) => b.classList.toggle("active", b.dataset.view === state.view));
  const views = { dashboard: renderDashboard, sessions: renderSessions, tasks: renderTasks, issues: renderIssues, changes: renderChanges };
  main.innerHTML = views[state.view]();
  if (state.view === "sessions" && state.sessionMode === "calendar") {
    if (!keepScroll && !render.scrolledOnce) {
      const firstBand = main.querySelector(".band");
      const target = firstBand ? firstBand.offsetTop - 40 : state.hourPx * 8;
      main.scrollTop = target;
      render.scrolledOnce = true;
    } else main.scrollTop = scroll;
  } else if (keepScroll) main.scrollTop = scroll;
  renderDetail();
}

function statusChips(list) {
  const counts = {};
  for (const s of list) counts[s.status] = (counts[s.status] || 0) + 1;
  return Object.keys(STATUS).map((k) =>
    `<button class="chip ${state.statusFilter.has(k) ? "active" : ""}" data-status="${k}"><span class="status ${k}">${STATUS[k].label}</span><span class="n">${counts[k] || 0}</span></button>`
  ).join("");
}

// ----- ダッシュボード -----
function renderDashboard() {
  const all = visibleSessions({ useStatus: false });
  if (!all.length) return emptyState();
  const t = now();
  const weekAgo = t - 7 * 86400;
  const running = all.filter((s) => s.status === "running");
  const waiting = all.filter((s) => s.status === "waiting");
  const weekActive = sum(all, (s) => sum(s.segments, ([a, b]) => (b > weekAgo ? Math.max(60, b - Math.max(a, weekAgo)) : 0)));
  const outTok = sum(all, (s) => s.tokens.output);
  const issues7 = sum(all, (s) => s.issues.filter((i) => i.ts > weekAgo).length);
  const commits7 = sum(all, (s) => s.commits.filter((c) => c.ts > weekAgo).length);

  const kpi = (label, value, sub = "") => `<div class="kpi"><div class="label">${label}</div><div class="value">${value}</div><div class="sub">${sub}</div></div>`;
  const activeList = [...running, ...waiting].sort((a, b) => (b.end || 0) - (a.end || 0));
  const recentIssues = all.flatMap((s) => s.issues.map((i) => ({ ...i, s }))).sort((a, b) => b.ts - a.ts).slice(0, 8);
  const recentSessions = [...all].sort((a, b) => (b.end || 0) - (a.end || 0)).slice(0, 8);

  return `
    <div class="kpis">
      ${kpi("セッション", all.length, `全期間`)}
      ${kpi("作業中", running.length, `入力待ち ${waiting.length}`)}
      ${kpi("直近7日の作業時間", fmtDur(weekActive), "記録の間隔10分以内を連続とみなす")}
      ${kpi("直近7日のコミット", commits7, "Bashでのgit commit")}
      ${kpi("直近7日の課題", issues7, "ツールエラー・中断・APIエラー")}
      ${kpi("出力トークン", fmtTok(outTok), `入力 ${fmtTok(sum(all, (s) => s.tokens.input + s.tokens.cacheRead + s.tokens.cacheCreate))}`)}
    </div>
    <div class="cards">
      <div class="card"><h3>日別の作業時間 <span class="muted">直近14日・作業種別</span></h3>${dailyChart(all)}</div>
      <div class="card"><h3>稼働中のセッション <span class="muted">${activeList.length}件</span></h3>
        ${activeList.length ? activeList.map(sessionRow).join("") : `<div class="empty">稼働中のセッションはありません</div>`}
      </div>
      <div class="card"><h3>ツール使用回数 <span class="muted">上位10</span></h3>${toolBars(all)}</div>
      <div class="card"><h3>プロジェクト別の作業時間</h3>${projectBars(all)}</div>
      <div class="card"><h3>最近のセッション</h3>${recentSessions.map(sessionRow).join("")}</div>
      <div class="card"><h3>最近の課題</h3>
        ${recentIssues.length ? recentIssues.map((i) => `<div class="rowlink" data-open="${i.s.id}">
          <span class="tag">${ISSUE_KIND[i.kind] || i.kind}</span><span class="grow">${esc(i.label)}: ${esc(i.text)}</span><span class="muted">${fmtAgo(i.ts)}</span></div>`).join("")
        : `<div class="empty">課題はありません</div>`}
      </div>
    </div>`;
}
function sessionRow(s) {
  return `<div class="rowlink" data-open="${s.id}">
    <i class="swatch" style="--c:${colorOf(s, "workType")}"></i>
    <span class="grow">${esc(s.title)}</span>${statusBadge(s)}<span class="muted num">${fmtAgo(s.end)}</span></div>`;
}

function dailyChart(list) {
  const days = 14;
  const today = new Date(); today.setHours(0, 0, 0, 0);
  const startDay = addDays(today, -(days - 1));
  const data = Array.from({ length: days }, (_, i) => ({ day: addDays(startDay, i), v: Object.fromEntries(WORK_TYPES.map((w) => [w, 0])) }));
  for (const s of list) {
    for (const [a, b0] of s.segments) {
      const b = Math.max(b0, a + 60);
      for (let i = 0; i < days; i++) {
        const d0 = data[i].day.getTime() / 1000, d1 = d0 + 86400;
        const ov = Math.min(b, d1) - Math.max(a, d0);
        if (ov > 0) data[i].v[s.workType] += ov / 3600;
      }
    }
  }
  const max = Math.max(0.5, ...data.map((d) => sum(WORK_TYPES, (w) => d.v[w])));
  const W = 560, H = 180, L = 30, B = 20, bw = (W - L) / days;
  const y = (h) => (H - B) - (h / max) * (H - B - 6);
  const ticks = niceTicks(max);
  let svg = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="日別の作業時間">`;
  for (const tv of ticks) svg += `<line x1="${L}" x2="${W}" y1="${y(tv)}" y2="${y(tv)}" stroke="var(--grid)"/><text x="${L - 4}" y="${y(tv) + 4}" text-anchor="end">${tv}h</text>`;
  data.forEach((d, i) => {
    const x = L + i * bw + 3, w = bw - 6;
    let acc = 0;
    const parts = WORK_TYPES.filter((wt) => d.v[wt] > 0);
    parts.forEach((wt, j) => {
      const h0 = acc, h1 = acc + d.v[wt];
      acc = h1;
      const top = y(h1), bot = y(h0) - (j > 0 ? 2 : 0); // 積み上げの間に 2px の隙間
      const hgt = Math.max(1, bot - top);
      const r = j === parts.length - 1 ? 3 : 0;
      svg += `<path d="${roundedTop(x, top, w, hgt, r)}" fill="${state.colorSlots.workType.get(wt)}"/>`;
    });
    const total = sum(WORK_TYPES, (wt) => d.v[wt]);
    const tip = `${d.day.getMonth() + 1}/${d.day.getDate()}(${WEEKDAYS[d.day.getDay()]}) 合計 ${fmtDur(total * 3600)}<br>` +
      WORK_TYPES.filter((wt) => d.v[wt] > 0).map((wt) => `<i class="swatch" style="--c:${state.colorSlots.workType.get(wt)}"></i> ${wt} ${fmtDur(d.v[wt] * 3600)}`).join("<br>");
    svg += `<rect x="${L + i * bw}" y="0" width="${bw}" height="${H - B}" fill="transparent" data-tip="${esc(tip)}"/>`;
    if (i % 2 === (days - 1) % 2) svg += `<text x="${x + w / 2}" y="${H - 5}" text-anchor="middle">${d.day.getMonth() + 1}/${d.day.getDate()}</text>`;
  });
  svg += `<line x1="${L}" x2="${W}" y1="${H - B}" y2="${H - B}" stroke="var(--axis)"/></svg>`;
  const legend = `<div class="legend" style="margin-top:6px">${WORK_TYPES.map((w) => `<span><i class="swatch" style="--c:${state.colorSlots.workType.get(w)}"></i>${w}</span>`).join("")}</div>`;
  return `<div class="chart">${svg}</div>${legend}`;
}
function roundedTop(x, y, w, h, r) {
  r = Math.min(r, h, w / 2);
  return `M${x},${y + h}V${y + r}Q${x},${y} ${x + r},${y}H${x + w - r}Q${x + w},${y} ${x + w},${y + r}V${y + h}Z`;
}
function niceTicks(max) {
  const steps = [0.5, 1, 2, 3, 4, 6, 8, 12];
  const step = steps.find((s) => max / s <= 4) || Math.ceil(max / 4);
  const out = [];
  for (let v = 0; v <= max + 1e-9; v += step) out.push(+v.toFixed(2));
  return out;
}
function hbars(rows, fmt = (v) => v) {
  if (!rows.length) return `<div class="empty">データがありません</div>`;
  const max = Math.max(...rows.map((r) => r[1]));
  return rows.map(([name, v]) => `<div class="hbar" data-tip="${esc(name)}: ${esc(fmt(v))}"><span class="name">${esc(name)}</span>
    <span><div class="bar" style="width:${(v / max) * 100}%"></div></span><span class="v">${esc(fmt(v))}</span></div>`).join("");
}
function toolBars(list) {
  const c = {};
  for (const s of list) for (const [k, v] of Object.entries(s.tools)) c[k] = (c[k] || 0) + v;
  return hbars(Object.entries(c).sort((a, b) => b[1] - a[1]).slice(0, 10));
}
function projectBars(list) {
  const c = {};
  for (const s of list) c[s.projectName] = (c[s.projectName] || 0) + s.activeSec;
  return hbars(Object.entries(c).sort((a, b) => b[1] - a[1]).slice(0, 8), fmtDur);
}
function emptyState() {
  return `<div class="empty">表示できるセッションがありません。Claude Code の履歴(projects/*.jsonl)が見つからないか、検索条件に一致しません。</div>`;
}

// ----- セッション -----
function renderSessions() {
  const base = visibleSessions({ useStatus: false });
  const list = visibleSessions();
  const toolbar = `
    <div class="toolbar">
      <div class="seg">
        <button data-mode="calendar" class="${state.sessionMode === "calendar" ? "active" : ""}">カレンダー</button>
        <button data-mode="list" class="${state.sessionMode === "list" ? "active" : ""}">一覧</button>
      </div>
      <span class="sep"></span>${statusChips(base)}
      <span class="sep"></span>
      <label>色分け <select id="colorBy">${Object.entries(COLOR_BY).map(([k, v]) => `<option value="${k}" ${k === state.colorBy ? "selected" : ""}>${v}</option>`).join("")}</select></label>
      ${state.sessionMode === "list" ? `<label>並び順 <select id="sort">
        <option value="recent" ${state.sort === "recent" ? "selected" : ""}>最終活動が新しい順</option>
        <option value="start" ${state.sort === "start" ? "selected" : ""}>開始が新しい順</option>
        <option value="active" ${state.sort === "active" ? "selected" : ""}>作業時間が長い順</option>
        <option value="status" ${state.sort === "status" ? "selected" : ""}>状態順</option></select></label>` : ""}
    </div>`;
  if (state.sessionMode === "list") return toolbar + legendHtml(list) + sessionTable(list);
  return toolbar + calendarHtml(list);
}

function sessionTable(list) {
  const sorters = {
    recent: (a, b) => (b.end || 0) - (a.end || 0),
    start: (a, b) => (b.start || 0) - (a.start || 0),
    active: (a, b) => b.activeSec - a.activeSec,
    status: (a, b) => STATUS[a.status].order - STATUS[b.status].order || (b.end || 0) - (a.end || 0),
  };
  const rows = [...list].sort(sorters[state.sort] || sorters.recent);
  if (!rows.length) return emptyState();
  return `<table class="list"><thead><tr>
    <th>状態</th><th>セッション</th><th>種別</th><th>プロジェクト</th><th>開始</th><th class="r">作業時間</th>
    <th class="r">発言</th><th class="r">ツール</th><th>タスク</th><th class="r">課題</th><th class="r">出力tok</th><th>最終活動</th></tr></thead><tbody>
    ${rows.map((s) => {
      const done = s.tasks.filter((t) => t.status === "completed").length;
      return `<tr data-open="${s.id}" class="${state.selected === s.id ? "selected" : ""}">
        <td>${statusBadge(s)}</td>
        <td><div class="title-cell"><i class="swatch" style="--c:${colorOf(s)}"></i><div><div class="clamp">${esc(s.title)}</div><div class="muted">${esc(s.branch)}</div></div></div></td>
        <td><span class="tag">${esc(s.workType)}</span></td>
        <td>${esc(s.projectName)}</td>
        <td class="num">${fmtDateTime(s.start)}</td>
        <td class="r">${fmtDur(s.activeSec)}</td>
        <td class="r">${s.promptCount}</td>
        <td class="r">${sum(Object.values(s.tools))}</td>
        <td>${s.tasks.length ? `<span class="progress"><i style="width:${(done / s.tasks.length) * 100}%"></i></span> <span class="num muted">${done}/${s.tasks.length}</span>` : `<span class="muted">-</span>`}</td>
        <td class="r">${s.issues.length || ""}</td>
        <td class="r">${fmtTok(s.tokens.output)}</td>
        <td class="muted">${fmtAgo(s.end)}</td></tr>`;
    }).join("")}</tbody></table>`;
}

function calendarHtml(list) {
  const ws = state.weekStart;
  const we = addDays(ws, 7);
  const wsT = ws.getTime() / 1000, weT = we.getTime() / 1000;
  const inWeek = list.filter((s) => s.segments.some(([a]) => a < weT) && effectiveEnd(s) >= wsT);
  const hp = state.hourPx;
  const H = hp * 24;
  const todayKey = new Date().toDateString();

  let html = `<div class="toolbar">
    <button class="btn" data-week="today">今週</button>
    <button class="btn" data-week="-1" aria-label="前の週">◀</button>
    <button class="btn" data-week="1" aria-label="次の週">▶</button>
    <strong class="num">${ws.getFullYear()}/${ws.getMonth() + 1}/${ws.getDate()} 〜 ${addDays(ws, 6).getMonth() + 1}/${addDays(ws, 6).getDate()}</strong>
    <span class="muted">${inWeek.length} セッション</span>
    <span class="sep"></span>
    <button class="btn" data-zoom="-1" aria-label="縮小">−</button><button class="btn" data-zoom="1" aria-label="拡大">＋</button>
    <span class="muted">帯 = 作業区間(記録の間隔が10分を超えたら分割)・斜線 = 作業中</span>
  </div>${legendHtml(inWeek)}`;

  html += `<div class="cal"><div class="cal-head"></div>`;
  for (let i = 0; i < 7; i++) {
    const d = addDays(ws, i);
    html += `<div class="cal-head ${d.toDateString() === todayKey ? "today" : ""}">${WEEKDAYS[d.getDay()]} ${d.getMonth() + 1}/${d.getDate()}</div>`;
  }
  html += `<div class="cal-hours" style="height:${H}px">${Array.from({ length: 23 }, (_, h) => `<div style="top:${(h + 1) * hp}px">${h + 1}:00</div>`).join("")}</div>`;

  for (let i = 0; i < 7; i++) {
    const d = addDays(ws, i);
    const d0 = d.getTime() / 1000, d1 = addDays(d, 1).getTime() / 1000;
    const isToday = d.toDateString() === todayKey;
    let col = `<div class="cal-day ${isToday ? "today" : ""}" style="height:${H}px">`;
    for (let h = 1; h < 24; h++) col += `<div class="cal-line" style="top:${h * hp}px"></div>`;
    if (hp >= 60) for (let h = 0; h < 24; h++) col += `<div class="cal-line half" style="top:${(h + 0.5) * hp}px"></div>`;

    // この日の帯を集めて、重なるものを横に並べる
    const items = [];
    for (const s of inWeek) {
      const segs = s.segments.map(([a, b]) => [a, b]);
      if (s.status === "running" && segs.length) segs[segs.length - 1][1] = effectiveEnd(s);
      for (const [a, b] of segs) {
        const st = Math.max(a, d0), en = Math.min(Math.max(b, a + 60), d1);
        if (en > st) items.push({ s, st, en, a, b });
      }
    }
    layoutLanes(items);
    for (const it of items) {
      const top = ((it.st - d0) / 3600) * hp;
      const height = Math.max(6, ((it.en - it.st) / 3600) * hp);
      const left = (it.lane / it.lanes) * 100, width = 100 / it.lanes;
      const s = it.s;
      const cls = ["band", s.status === "running" ? "running" : "", state.selected === s.id ? "selected" : "", state.recentlyUpdated.has(s.id) ? "flash" : ""].join(" ");
      const tip = `<b>${esc(s.title)}</b><br>${STATUS[s.status].label} ・ ${esc(s.workType)} ・ ${esc(s.projectName)}<br>${fmtTime(it.a)}〜${fmtTime(it.b)}(${fmtDur(it.b - it.a)})`;
      col += `<div class="${cls}" data-open="${s.id}" data-tip="${esc(tip)}" style="--c:${colorOf(s)};top:${top}px;height:${height}px;left:calc(${left}% + 1px);width:calc(${width}% - 2px)">${height >= 14 ? `<span class="t">${esc(s.title)}</span>` : ""}</div>`;
    }
    if (isToday) {
      const n = new Date();
      col += `<div class="cal-now" style="top:${(n.getHours() + n.getMinutes() / 60) * hp}px"></div>`;
    }
    html += col + `</div>`;
  }
  return html + `</div>`;
}
function layoutLanes(items) {
  // 重なりのまとまり(クラスタ)ごとに、空いている最小の列へ割り当てる
  items.sort((x, y) => x.st - y.st || y.en - x.en);
  let cluster = [], clusterEnd = -Infinity;
  const flush = () => {
    const lanes = Math.max(1, ...cluster.map((c) => c.lane + 1));
    cluster.forEach((c) => (c.lanes = lanes));
    cluster = [];
  };
  const laneEnds = [];
  for (const it of items) {
    if (it.st >= clusterEnd && cluster.length) { flush(); laneEnds.length = 0; }
    let lane = laneEnds.findIndex((e) => e <= it.st);
    if (lane < 0) { lane = laneEnds.length; laneEnds.push(it.en); } else laneEnds[lane] = it.en;
    it.lane = lane;
    cluster.push(it);
    clusterEnd = Math.max(clusterEnd, it.en);
  }
  if (cluster.length) flush();
}

// ----- タスク -----
function renderTasks() {
  let list = visibleSessions({ useStatus: false });
  if (state.taskScope === "active") list = list.filter((s) => s.status === "running" || s.status === "waiting");
  const cols = { pending: [], in_progress: [], completed: [] };
  for (const s of list) for (const t of s.tasks) (cols[t.status] || cols.pending).push({ t, s });
  for (const k in cols) cols[k].sort((a, b) => (b.t.ts || 0) - (a.t.ts || 0));
  const total = sum(Object.values(cols), (c) => c.length);
  const toolbar = `<div class="toolbar">
    <div class="seg"><button data-taskscope="all" class="${state.taskScope === "all" ? "active" : ""}">全セッション</button>
    <button data-taskscope="active" class="${state.taskScope === "active" ? "active" : ""}">稼働中のみ</button></div>
    <label>色分け <select id="colorBy">${Object.entries(COLOR_BY).map(([k, v]) => `<option value="${k}" ${k === state.colorBy ? "selected" : ""}>${v}</option>`).join("")}</select></label>
    <span class="muted">Claude が TaskCreate/TaskUpdate・TodoWrite で管理したタスク(各セッションの最新の状態)</span></div>`;
  if (!total) return toolbar + `<div class="empty">タスクの記録はありません。</div>`;
  const col = (key, label) => `<div class="column"><h3><span>${label}</span><span class="num muted">${cols[key].length}</span></h3>
    ${cols[key].map(({ t, s }) => `<div class="tcard" data-open="${s.id}" style="--c:${colorOf(s)}">
      <div>${esc(t.content)}</div><div class="meta">${statusBadge(s)} ・ ${esc(s.title)}</div></div>`).join("")}</div>`;
  return toolbar + `<div class="board">${col("pending", "未着手")}${col("in_progress", "進行中")}${col("completed", "完了")}</div>`;
}

// ----- 課題 -----
function renderIssues() {
  const list = visibleSessions({ useStatus: false });
  const all = list.flatMap((s) => s.issues.map((i) => ({ ...i, s })));
  const counts = {};
  for (const i of all) counts[i.kind] = (counts[i.kind] || 0) + 1;
  const rows = all.filter((i) => !state.issueFilter.size || state.issueFilter.has(i.kind)).sort((a, b) => b.ts - a.ts);
  const toolbar = `<div class="toolbar">${Object.entries(ISSUE_KIND).map(([k, v]) =>
    `<button class="chip ${state.issueFilter.has(k) ? "active" : ""}" data-issue="${k}">${v}<span class="n">${counts[k] || 0}</span></button>`).join("")}
    <span class="muted">履歴から機械的に抽出(ツール結果の is_error、ユーザーによる中断、APIエラー)</span></div>`;
  if (!rows.length) return toolbar + `<div class="empty">課題はありません。</div>`;
  return toolbar + `<table class="list"><thead><tr><th>日時</th><th>種類</th><th>ツール等</th><th>内容</th><th>セッション</th></tr></thead><tbody>
    ${rows.slice(0, 500).map((i) => `<tr data-open="${i.s.id}">
      <td class="num">${fmtDateTime(i.ts)}</td><td><span class="tag">${ISSUE_KIND[i.kind] || i.kind}</span></td>
      <td>${esc(i.label)}${i.detail ? `<div class="muted clamp">${esc(i.detail)}</div>` : ""}</td>
      <td><div class="clamp">${esc(i.text)}</div></td>
      <td><div class="title-cell"><i class="swatch" style="--c:${colorOf(i.s)}"></i><span class="clamp">${esc(i.s.title)}</span></div></td></tr>`).join("")}
    </tbody></table>`;
}

// ----- 変更履歴 -----
function renderChanges() {
  const list = visibleSessions({ useStatus: false });
  const commits = list.flatMap((s) => s.commits.map((c) => ({ ...c, s }))).sort((a, b) => b.ts - a.ts);
  const files = new Map();
  for (const s of list) for (const f of s.files) {
    const e = files.get(f.path) || { path: f.path, edits: 0, last: 0, sessions: new Set() };
    e.edits += f.edits; e.last = Math.max(e.last, f.last || 0); e.sessions.add(s.id);
    files.set(f.path, e);
  }
  const fileRows = [...files.values()].sort((a, b) => b.last - a.last).slice(0, 300);
  return `<div class="cards">
    <div class="card"><h3>コミット <span class="muted">${commits.length}件(結果にコミットハッシュが出たもの)</span></h3>
      ${commits.length ? `<table class="list"><thead><tr><th>日時</th><th>ハッシュ</th><th>メッセージ</th><th>セッション</th></tr></thead><tbody>
      ${commits.map((c) => `<tr data-open="${c.s.id}"><td class="num">${fmtDateTime(c.ts)}</td><td class="num">${esc(c.hash)}<div class="muted">${esc(c.branch)}</div></td>
        <td style="min-width:120px">${esc(c.message || "-")}</td><td><span class="clamp">${esc(c.s.title)}</span></td></tr>`).join("")}</tbody></table>`
      : `<div class="empty">コミットの記録はありません。</div>`}</div>
    <div class="card"><h3>変更したファイル <span class="muted">${files.size}件(Edit/Write等)</span></h3>
      ${fileRows.length ? `<table class="list"><thead><tr><th>ファイル</th><th class="r">編集</th><th class="r">セッション</th><th>最終</th></tr></thead><tbody>
      ${fileRows.map((f) => `<tr data-open="${[...f.sessions][0]}"><td style="overflow-wrap:anywhere">${esc(f.path)}</td><td class="r">${f.edits}</td><td class="r">${f.sessions.size}</td><td class="muted">${fmtAgo(f.last)}</td></tr>`).join("")}
      </tbody></table>` : `<div class="empty">ファイル変更の記録はありません。</div>`}</div></div>`;
}

// ----- 詳細パネル -----
function renderDetail() {
  const box = $("#detail");
  const s = state.selected && state.sessions.get(state.selected);
  if (!s) { box.classList.add("hidden"); return; }
  box.classList.remove("hidden");
  const logScroll = $("#log", box)?.scrollTop;
  const done = s.tasks.filter((t) => t.status === "completed").length;
  const tools = Object.entries(s.tools).sort((a, b) => b[1] - a[1]);
  const mark = { completed: "☑", in_progress: "◐", pending: "☐" };
  box.innerHTML = `
    <button class="close" data-close aria-label="閉じる">×</button>
    <h2>${esc(s.title)}</h2>
    <div class="toolbar">${statusBadge(s)}<span class="tag"><i class="swatch" style="--c:${colorOf(s, "workType")}"></i>${esc(s.workType)}</span>
      <span class="muted">${fmtAgo(s.end)}に最終活動</span></div>

    <div class="box"><h4>セッションの状態</h4><dl class="meta">
      <dt>状態</dt><dd>${STATUS[s.status].label}</dd>
      <dt>判定の根拠</dt><dd>${s.statusSource === "process" ? "起動中のClaude Codeプロセスの状態" : "履歴の末尾から推定(プロセス情報なし)"}</dd>
      <dt>最後の出来事</dt><dd>${esc(LAST_KIND[s.lastKind] || "-")}</dd>
      <dt>タスク</dt><dd>${s.tasks.length ? `${done}/${s.tasks.length} 完了` : "なし"}</dd>
      <dt>課題</dt><dd>${s.issues.length} 件</dd>
    </dl></div>

    <div class="box"><h4>概要</h4><dl class="meta">
      <dt>プロジェクト</dt><dd>${esc(s.project)}</dd>
      <dt>ブランチ</dt><dd>${esc(s.branch || "-")}</dd>
      <dt>開始</dt><dd class="num">${fmtDateTime(s.start)}</dd>
      <dt>最終活動</dt><dd class="num">${fmtDateTime(s.end)}</dd>
      <dt>作業時間</dt><dd>${fmtDur(s.activeSec)}(${s.segments.length}区間)</dd>
      <dt>モデル</dt><dd>${esc(Object.keys(s.models).join(", ") || "-")}</dd>
      <dt>トークン</dt><dd class="num">出力 ${fmtTok(s.tokens.output)} / 入力 ${fmtTok(s.tokens.input)} / キャッシュ読込 ${fmtTok(s.tokens.cacheRead)} / キャッシュ作成 ${fmtTok(s.tokens.cacheCreate)}</dd>
      <dt>応答数</dt><dd>${s.assistantMessages}${s.subagentRecords ? `(サブエージェントの記録 ${s.subagentRecords} 件を含む)` : ""}</dd>
      <dt>起動元</dt><dd>${esc(s.entrypoint || "-")} ・ v${esc(s.version || "-")}</dd>
      <dt>ID</dt><dd class="muted">${esc(s.id)}</dd>
    </dl></div>

    ${s.tasks.length ? `<div class="box"><h4>タスク(${esc(s.taskSource)})</h4><ul class="plain">
      ${s.tasks.map((t) => `<li><span class="task-st" title="${esc(t.status)}">${mark[t.status] || "☐"}</span> ${esc(t.content)}</li>`).join("")}</ul></div>` : ""}

    ${s.commits.length ? `<div class="box"><h4>コミット ${s.commits.length}件</h4><ul class="plain">
      ${s.commits.map((c) => `<li><span class="num">${esc(c.hash)}</span> ${esc(c.message)} <span class="muted">${fmtTime(c.ts)}</span></li>`).join("")}</ul></div>` : ""}

    ${s.files.length ? `<div class="box"><h4>変更したファイル ${s.files.length}件</h4><ul class="plain">
      ${s.files.slice(0, 40).map((f) => `<li>${esc(f.path)} <span class="muted">×${f.edits}</span></li>`).join("")}</ul></div>` : ""}

    <div class="box"><h4>ユーザーの発言 ${s.promptCount}件</h4><ul class="plain">
      ${s.prompts.slice(-30).reverse().map((p) => `<li><span class="muted num">${fmtDateTime(p.ts)}</span><br>${esc(p.text)}</li>`).join("") || `<li class="muted">なし</li>`}</ul></div>

    ${tools.length ? `<div class="box"><h4>ツール使用回数</h4>${hbars(tools.slice(0, 12))}</div>` : ""}

    ${s.issues.length ? `<div class="box"><h4>課題 ${s.issues.length}件</h4><ul class="plain">
      ${s.issues.slice(-20).reverse().map((i) => `<li><span class="tag">${ISSUE_KIND[i.kind] || i.kind}</span> ${esc(i.label)} <span class="muted">${fmtTime(i.ts)}</span><br>${esc(i.text)}</li>`).join("")}</ul></div>` : ""}

    <div class="box"><h4>会話ログ <button class="btn" data-log style="float:right">${state.logOpen ? "閉じる" : "表示"}</button></h4>
      <div id="log" class="log" style="max-height:480px;overflow:auto">${state.logOpen ? (state.logHtml || "読み込み中…") : ""}</div></div>`;
  if (logScroll !== undefined && $("#log", box)) $("#log", box).scrollTop = logScroll;
}

async function loadLog(id) {
  const res = await fetch(`/api/session/${encodeURIComponent(id)}/log?limit=300`);
  const data = await res.json();
  if (state.selected !== id) return;
  const label = { prompt: "ユーザー", text: "Claude", tool_use: "ツール実行", tool_result: "ツール結果", error: "ツールエラー" };
  state.logHtml = data.events.map((e) => `<div class="ev ${e.kind} ${e.side ? "side" : ""}"><span class="h">${fmtTime(e.ts)} ${label[e.kind] || e.kind}${e.tool ? " " + esc(e.tool) : ""}${e.side ? "(サブエージェント)" : ""}</span>${esc(e.text)}</div>`).join("") || "記録がありません";
  const log = $("#log");
  if (log) {
    const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 40;
    log.innerHTML = state.logHtml;
    if (atBottom) log.scrollTop = log.scrollHeight;
  }
}

function openSession(id) {
  if (state.selected !== id) { state.logOpen = false; state.logHtml = ""; }
  state.selected = id;
  render({ keepScroll: true });
}

// ---------- 操作 ----------
document.addEventListener("click", (e) => {
  const t = e.target.closest("button, [data-open]");
  if (!t) return;
  const d = t.dataset;
  if (d.view) { state.view = d.view; savePrefs(); render(); return; }
  if (d.mode) { state.sessionMode = d.mode; render.scrolledOnce = false; savePrefs(); render(); return; }
  if (d.status) { state.statusFilter.has(d.status) ? state.statusFilter.delete(d.status) : state.statusFilter.add(d.status); render({ keepScroll: true }); return; }
  if (d.issue) { state.issueFilter.has(d.issue) ? state.issueFilter.delete(d.issue) : state.issueFilter.add(d.issue); render({ keepScroll: true }); return; }
  if (d.taskscope) { state.taskScope = d.taskscope; savePrefs(); render(); return; }
  if (d.week) {
    state.weekStart = d.week === "today" ? startOfWeek(new Date()) : addDays(state.weekStart, 7 * Number(d.week));
    render.scrolledOnce = false; render(); return;
  }
  if (d.zoom) { state.hourPx = Math.min(120, Math.max(16, state.hourPx + Number(d.zoom) * 8)); savePrefs(); render({ keepScroll: true }); return; }
  if (d.close !== undefined) { state.selected = null; state.logOpen = false; render({ keepScroll: true }); return; }
  if (d.log !== undefined) { state.logOpen = !state.logOpen; renderDetail(); if (state.logOpen) loadLog(state.selected); return; }
  if (d.open) openSession(d.open);
});
document.addEventListener("change", (e) => {
  if (e.target.id === "colorBy") { state.colorBy = e.target.value; savePrefs(); render({ keepScroll: true }); }
  if (e.target.id === "sort") { state.sort = e.target.value; savePrefs(); render({ keepScroll: true }); }
});
$("#search").addEventListener("input", (e) => { state.search = e.target.value; render({ keepScroll: true }); });
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && state.selected) { state.selected = null; state.logOpen = false; render({ keepScroll: true }); }
});
// Ctrl+ホイールでカレンダーを拡大縮小
$("#main").addEventListener("wheel", (e) => {
  if (!e.ctrlKey || state.view !== "sessions" || state.sessionMode !== "calendar") return;
  e.preventDefault();
  state.hourPx = Math.min(120, Math.max(16, state.hourPx + (e.deltaY < 0 ? 8 : -8)));
  savePrefs(); render({ keepScroll: true });
}, { passive: false });

// ツールチップ
const tip = $("#tooltip");
document.addEventListener("mousemove", (e) => {
  const t = e.target.closest("[data-tip]");
  if (!t) { tip.classList.add("hidden"); return; }
  tip.innerHTML = t.dataset.tip;
  tip.classList.remove("hidden");
  const r = tip.getBoundingClientRect();
  let x = e.clientX + 14, y = e.clientY + 14;
  if (x + r.width > innerWidth - 8) x = e.clientX - r.width - 14;
  if (y + r.height > innerHeight - 8) y = e.clientY - r.height - 14;
  tip.style.left = x + "px"; tip.style.top = y + "px";
});

// 「〜分前」や現在時刻線を進めるため、30秒ごとに描き直す(サーバーへの問い合わせはしない)
setInterval(() => {
  if (document.activeElement?.tagName === "SELECT") return; // 選択中のプルダウンを閉じないため
  render({ keepScroll: true });
}, 30000);

loadState().then(connectEvents).catch((err) => {
  $("#main").innerHTML = `<div class="empty">サーバーからデータを取得できませんでした: ${esc(err)}</div>`;
});
