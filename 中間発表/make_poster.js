// 中間発表ポスター(A0縦・2段組)を生成する
// 実行(i22satou/ で、pptxgenjs を npm install した環境):
//   node 中間発表/make_poster.js 中間発表/2026-11-05_中間発表ポスター.pptx 中間発表/図/poster
const path = require("path");
const pptxgen = require("pptxgenjs");

const OUT = process.argv[2];
const FIG = process.argv[3]; // 中間発表/図/poster

const W = 33.11, H = 46.81; // A0 縦(インチ)
const M = 1.0, GAP = 0.8;
const CW = (W - 2 * M - GAP) / 2; // 段の幅
const X1 = M, X2 = M + CW + GAP;

const FONT = "Yu Gothic";
const C = {
  ink: "1F2328", muted: "5B6168", teal: "0F4C5C", tealTint: "E8F1F2",
  amber: "E8A33D", amberTint: "FCF1DE", line: "C9D3D6", white: "FFFFFF",
};
const SIZE = {
  "poster_処理の流れ.png": [3376, 3955], "poster_式_Neff.png": [1201, 392],
  "poster_推定軌跡_1441_1442.png": [3700, 3374], "poster_曲がり判定_1441_1442.png": [3659, 3082],
  "poster_経路帯.png": [2931, 1193], "poster_誤差の累積.png": [3974, 1590],
};

const pres = new pptxgen();
pres.defineLayout({ name: "A0P", width: W, height: H });
pres.layout = "A0P";
pres.theme = { headFontFace: FONT, bodyFontFace: FONT };
pres.title = "中間発表ポスター";
const s = pres.addSlide();
s.background = { color: C.white };

const T = (text, o) => s.addText(text, { isTextBox: true, fontFace: FONT, color: C.ink, margin: 0, valign: "top", ...o });
const img = (file, x, y, w, extra = {}) => {
  const [pw, ph] = SIZE[file];
  const h = (w * ph) / pw;
  s.addImage({ path: path.join(FIG, file), x, y, w, h, ...extra });
  return h;
};
// 節の見出し: 番号の丸 + 見出し文字
const section = (x, y, num, title) => {
  s.addShape(pres.shapes.OVAL, { x, y, w: 1.15, h: 1.15, fill: { color: C.amber }, line: { color: C.amber } });
  T(String(num), { x, y, w: 1.15, h: 1.15, fontSize: 48, bold: true, color: C.white, align: "center", valign: "middle" });
  T(title, { x: x + 1.45, y: y - 0.05, w: CW - 1.5, h: 1.25, fontSize: 56, bold: true, color: C.teal, valign: "middle" });
  return y + 1.6;
};
const sub = (x, y, text, w = CW) => {
  T(text, { x, y, w, h: 0.7, fontSize: 36, bold: true, color: C.teal });
  return y + 0.85;
};
const bullets = (items, o) =>
  T(items.map((t, i) => ({ text: t, options: { bullet: { indent: 30 }, breakLine: i < items.length - 1, paraSpaceAfter: 10 } })),
    { fontSize: 30, lineSpacingMultiple: 1.15, ...o });
const caption = (text, x, y, w) => T(text, { x, y, w, h: 0.5, fontSize: 22, color: C.muted, align: "center" });

// ---------------- タイトル ----------------
s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: M, y: 0.8, w: W - 2 * M, h: 5.6, fill: { color: C.teal }, line: { color: C.teal }, rectRadius: 0.3 });
T("令和8年度　徳山工業高等専門学校　情報電子工学科　卒業研究中間発表", { x: M + 0.8, y: 1.25, w: W - 2 * M - 1.6, h: 0.7, fontSize: 32, color: "CFE3E6" });
T("地図制約付き移動様態適応パーティクルフィルタによる\nスマートフォン屋内歩行者測位",
  { x: M + 0.8, y: 2.05, w: W - 2 * M - 1.6, h: 2.9, fontSize: 84, bold: true, color: C.white, lineSpacingMultiple: 1.05 });
T("佐藤 壮真（浦上研究室）", { x: M + 0.8, y: 5.15, w: W - 2 * M - 1.6, h: 0.9, fontSize: 44, color: C.white });

// ---------------- 左の段 ----------------
let y = 7.0;
y = section(X1, y, 1, "研究背景");
bullets([
  "屋内ではGNSS(GPS)の信号が弱く，測位が難しい[1]",
  "Wi-Fi・BLEビーコンは，発信機の設置や維持にコストがかかる[2]",
  "スマートフォンのセンサで位置を積み上げる歩行者推測航法(PDR)は設備不要だが，歩くほど誤差が累積する[2]",
], { x: X1, y, w: CW, h: 3.1 });
y += 3.2;
let h = img("poster_誤差の累積.png", X1 + 1.8, y, CW - 3.6);
y += h + 0.05;
caption("方位が3度ずれると，30m歩いた時点で推定が壁を通り抜ける(計算例)", X1, y, CW);
y += 0.9;

y = section(X1, y, 2, "目的");
s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: X1, y, w: CW, h: 1.75, fill: { color: C.tealTint }, line: { color: C.tealTint }, rectRadius: 0.2 });
T("追加の設備なしで，建物の地図だけでPDRの誤差を抑える", { x: X1 + 0.4, y: y + 0.1, w: CW - 0.8, h: 1.55, fontSize: 38, bold: true, color: C.teal, valign: "middle" });
y += 2.05;
T("移動様態適応PF[4]（直進・曲がりで粒子数とノイズを変える方式）に，次の2点を加える", { x: X1, y, w: CW, h: 1.1, fontSize: 30 });
y += 1.3;
// 提案カード
const cardW = (CW - 0.5) / 2, cardH = 7.5;
const card = (x, title, body) => {
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w: cardW, h: cardH, fill: { color: C.amberTint }, line: { color: C.amber, width: 2 }, rectRadius: 0.2 });
  T(title, { x: x + 0.35, y: y + 0.3, w: cardW - 0.7, h: 1.4, fontSize: 36, bold: true, color: C.teal });
  T(body, { x: x + 0.35, y: y + 4.95, w: cardW - 0.7, h: cardH - 5.1, fontSize: 26, lineSpacingMultiple: 1.1 });
};
card(X1, "提案1\n経路帯の自動抽出", "二値化した平面図から通路を自動で抽出し，経路帯の外に出た粒子の重みを下げる．経路の手入力は不要");
img("poster_経路帯.png", X1 + 0.25, y + 1.95, cardW - 0.5);
card(X1 + cardW + 0.5, "提案2\n不確実性適応粒子数", [{ text: "前の歩の比率 r = N" }, { text: "eff", options: { subscript: true } }, { text: "/N が0.30未満なら粒子数を1.5倍，0.60超なら0.75倍(80〜1200個)" }]);
img("poster_式_Neff.png", X1 + cardW + 0.5 + 0.9, y + 2.1, cardW - 1.8);
T([{ text: "N" }, { text: "eff", options: { subscript: true } }, { text: "：有効粒子数(効いている粒子の数)[5]" }], { x: X1 + cardW + 0.85, y: y + 3.95, w: cardW - 0.7, h: 0.9, fontSize: 22, color: C.muted });
y += cardH + 0.5;

y = section(X1, y, 3, "方法");
h = img("poster_処理の流れ.png", X1, y, 8.7);
const sideX = X1 + 9.0, sideW = CW - 9.0;
T([
  { text: "PDR", options: { bold: true, color: C.teal, breakLine: true } },
  { text: "SmartPDR[3]で歩を検出し，歩幅と方位から移動量を求める．歩調の上限2.9Hzで過検出を防ぎ，歩幅に校正ゲイン(暫定2.10)を掛ける", options: { breakLine: true } },
  { text: " ", options: { breakLine: true, fontSize: 14 } },
  { text: "移動様態", options: { bold: true, color: C.teal, breakLine: true } },
  { text: "直近1.5秒の方位変化とヨーレート(75%値)で，直進／曲がりを判定する", options: { breakLine: true } },
  { text: " ", options: { breakLine: true, fontSize: 14 } },
  { text: "PF", options: { bold: true, color: C.teal, breakLine: true } },
  { text: "1歩ごとに粒子を動かし，壁を横切った粒子の重みを0にする[4]．壁に近い粒子と，経路帯の外の粒子は重みを下げる" },
], { x: sideX, y: y + 0.2, w: sideW, h: h - 0.2, fontSize: 26, lineSpacingMultiple: 1.12 });
y += h + 0.4;

// 参考文献(左の段の下)
T([
  "[1] 目黒淳一，竹内栄二朗，鈴木太郎：ロボティクスにおけるGNSS失敗学，日本ロボット学会誌，Vol.37，No.7，pp.585–592，2019．",
  "[2] 西尾信彦：屋内測位技術の概要と動向，電気設備学会誌，Vol.42，No.7，pp.393–396，2022．",
  "[3] W. Kang and Y. Han: SmartPDR: Smartphone-Based Pedestrian Dead Reckoning for Indoor Localization, IEEE Sensors Journal, Vol.15, No.5, pp.2906–2916, 2015.",
  "[4] 秋山高行，中原豪，山崎勝也，大橋洋輝，佐藤暁子：移動様態に応じたパーティクルフィルタによる歩行者自律測位方式の提案と評価，FIT2013，RO-017，pp.165–171，2013．",
  "[5] M. S. Arulampalam, S. Maskell, N. Gordon and T. Clapp: A Tutorial on Particle Filters for Online Nonlinear/Non-Gaussian Bayesian Tracking, IEEE Trans. Signal Processing, Vol.50, No.2, pp.174–188, 2002.",
].map((t, i, a) => ({ text: t, options: { breakLine: i < a.length - 1 } })),
{ x: X1, y: H - M - 3.0, w: CW, h: 3.0, fontSize: 16, color: C.muted, valign: "bottom", lineSpacingMultiple: 1.05 });
const LEFT_END = y;

// ---------------- 右の段 ----------------
y = 7.0;
y = section(X2, y, 4, "進捗");
y = sub(X2, y, "実装・改善したこと");
const statW = (CW - 0.8) / 3;
[
  ["±5%以内", "検出した歩数の誤差\n(過検出を是正)"],
  ["15.6→2.4回", "粒子の全滅回数\n(3本の平均)"],
  ["提案1・2", "実装して既定で\n有効にした"],
].forEach(([big, label], i) => {
  const x = X2 + i * (statW + 0.4);
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w: statW, h: 2.65, fill: { color: C.tealTint }, line: { color: C.tealTint }, rectRadius: 0.2 });
  T(big, { x, y: y + 0.25, w: statW, h: 1.2, fontSize: 50, bold: true, color: C.teal, align: "center", valign: "middle" });
  T(label, { x, y: y + 1.4, w: statW, h: 1.1, fontSize: 24, color: C.ink, align: "center" });
});
y += 3.0;

y = sub(X2, y, "4方式の比較(実測データ 記録1441・1442)");
h = img("poster_推定軌跡_1441_1442.png", X2 + 1.3, y, CW - 2.6);
y += h + 0.05;
caption("推定軌跡(乱数の初期値42の1回分)", X2, y, CW);
y += 0.65;

const hdr = (t) => ({ text: t, options: { bold: true, color: C.white, fill: { color: C.teal }, align: "center", valign: "middle" } });
const cell = (t, o = {}) => ({ text: t, options: { align: "center", valign: "middle", ...o } });
const best = { bold: true, color: C.teal };
const H2 = (t, o = {}) => ({ text: t, options: { bold: true, color: C.white, fill: { color: C.teal }, align: "center", valign: "middle", ...o } });
s.addTable([
  [H2("方式", { rowspan: 2 }), H2("記録1441", { colspan: 2 }), H2("記録1442", { colspan: 2 })],
  [H2("終点誤差[m]"), H2("全滅[回]"), H2("終点誤差[m]"), H2("全滅[回]")],
  [cell("A PDRのみ", { align: "left" }), cell("18.46"), cell("－"), cell("13.32"), cell("－")],
  [cell("B 固定粒子数PF(600個)", { align: "left" }), cell("17.49±1.39"), cell("0.00"), cell("19.02±6.84"), cell("2.67")],
  [cell("C 移動様態適応PF[4]", { align: "left" }), cell("17.16±0.53"), cell("0.00"), cell("13.29±1.54"), cell("1.50")],
  [cell("E 提案方式(C＋提案1・2)", { align: "left", bold: true }), cell("15.75±0.43", best), cell("0.00"), cell("12.57±1.21", best), cell("0.17")],
], {
  x: X2, y, w: CW, colW: [4.75, 2.75, 1.9, 2.75, 1.9], rowH: 0.58, fontFace: FONT, fontSize: 22, color: C.ink,
  border: { type: "solid", pt: 1, color: C.line }, margin: 0.06,
});
y += 0.58 * 6 + 0.2;
T("6シードの平均±標準偏差(Aは乱数を使わないので1回)．2本は条件の調整に用いたデータで評価用ではない．時刻と対応した正解位置がないため，RMSEの代わりに終点誤差(推定の終点と申告した終点の距離)を見た",
  { x: X2, y, w: CW, h: 1.2, fontSize: 20, color: C.muted, lineSpacingMultiple: 1.05 });
y += 1.35;
T([
  { text: "提案方式Eは2本とも通路に沿い，終点誤差が最も小さかった．", options: { bold: true } },
  { text: "ただし，Cとの差は0.7〜1.4mと小さい" },
], { x: X2, y, w: CW, h: 1.2, fontSize: 28, lineSpacingMultiple: 1.1 });
y += 1.45;

y = sub(X2, y, "分かった課題：移動様態の判定が「曲がり」から戻らない");
h = img("poster_曲がり判定_1441_1442.png", X2, y, 9.6);
T([
  { text: "実際の曲がり角は2か所だが，歩の85〜100%が「曲がり」と判定された", options: { bold: true, breakLine: true } },
  { text: " ", options: { breakLine: true, fontSize: 12 } },
  { text: "直進中も歩行の揺れでヨーレートが15〜19度/秒あり，曲がり終了の条件(10度/秒未満)を下回らない", options: { breakLine: true } },
  { text: " ", options: { breakLine: true, fontSize: 12 } },
  { text: "→ 今は，移動様態に合わせる効果(BとCの差)を示せていない" },
], { x: X2 + 9.9, y: y + 0.3, w: CW - 9.9, h: h - 0.3, fontSize: 25, lineSpacingMultiple: 1.1 });
y += h + 0.5;

y = section(X2, y, 5, "今後の予定");
const steps = [
  ["追加計測", "校正用の直線歩行と，地点ごとに時刻を記録する評価用の歩行"],
  ["条件の決め直し", "歩幅の校正ゲインと曲がり判定のしきい値を校正用データで決める"],
  ["RMSEで比較", "正解位置を使い4方式と提案1・2の効果を確かめる(11月末まで)"],
  ["卒業論文", "12月に全章の初稿"],
];
const stW = (CW - 3 * 0.45) / 4, stH = H - M - y;
steps.forEach(([t, d], i) => {
  const x = X2 + i * (stW + 0.45);
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w: stW, h: stH, fill: { color: i === 3 ? C.tealTint : C.amberTint }, line: { color: i === 3 ? C.teal : C.amber, width: 2 }, rectRadius: 0.15 });
  T(t, { x: x + 0.15, y: y + 0.2, w: stW - 0.3, h: 0.7, fontSize: 28, bold: true, color: C.teal, align: "center" });
  T(d, { x: x + 0.2, y: y + 0.95, w: stW - 0.4, h: stH - 1.1, fontSize: 21, lineSpacingMultiple: 1.05 });
  if (i < 3) s.addShape(pres.shapes.RIGHT_ARROW, { x: x + stW + 0.05, y: y + stH / 2 - 0.3, w: 0.35, h: 0.6, fill: { color: C.amber }, line: { color: C.amber } });
});

console.log("左の段の終わり:", LEFT_END.toFixed(2), "右の段 今後の予定の高さ:", stH.toFixed(2));
pres.writeFile({ fileName: OUT }).then(() => console.log("保存:", OUT));
