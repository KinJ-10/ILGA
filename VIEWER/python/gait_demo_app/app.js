"use strict";
const el = id => document.getElementById(id);
let token = "", current = null, busy = false;
const labels = {idle:"待機中", connecting:"接続中", recording:"受信中", stopping:"停止中", analyzing:"解析中", completed:"完了", failed:"確認が必要"};
function observed() {
  return {actual_steps:el("actual").value, start_foot:el("startFoot").value,
    end_foot:el("endFoot").value, sync_pre:el("pre").value, sync_post:el("post").value};
}
function display(s) {
  current = s;
  el("state").textContent = labels[s.state] || s.state;
  el("detail").textContent = s.detail;
  el("counts").textContent = s.samples + "件 / データ欠損 " + s.missing + "件 / " + s.elapsed + "秒" + (s.effective_hz === null ? "" : " / 約" + s.effective_hz + "Hz");
  el("counts").textContent += " / 無効センサー " + s.invalid_sensor_samples + "件 / 受信形式不正 " + s.invalid_payloads + "件 / センサー異常 " + (s.sensor_fault ? "あり" : "なし");
  el("progress").max = s.duration;
  el("progress").value = s.elapsed;
  const active = ["connecting","recording","stopping","analyzing"].includes(s.state);
  el("start").disabled = busy || active;
  el("markStart").disabled = busy || s.state !== "recording" || s.markers.length !== 0;
  el("markFinish").disabled = busy || s.state !== "recording" || s.markers.length !== 1;
  el("stop").disabled = busy || !["connecting","recording"].includes(s.state);
  el("save").disabled = busy || !["recording","completed","failed"].includes(s.state);
  el("result").hidden = !s.report;
  el("savePath").textContent = s.save_path || "保存先を確認できません。";
  el("openFolder").disabled = busy || !s.save_path;
  if (s.report) el("report").href = s.report;
  if (s.error) showError(s.error);
}
function showError(message) { el("error").textContent = message; el("error").hidden = false; }
async function send(action, data = {}) {
  if (busy) return;
  busy = true;
  if (current) display(current);
  el("error").hidden = true;
  try {
    const r = await fetch("/api/" + action, {method:"POST", headers:{"Content-Type":"application/json","X-ILGA-Token":token},
      body:JSON.stringify({run_id:current?.run_id, ...data})});
    const s = await r.json();
    if (!r.ok) throw new Error(s.error);
    display(s);
    if (s.message) el("folderMessage").textContent = s.message;
  } catch (e) { showError(e.message || "接続を確認してください。"); }
  finally { busy = false; if (current) display(current); }
}
el("start").onclick = () => send("start", {trial_name:el("trial").value, duration:el("duration").value, ...observed()});
el("markStart").onclick = () => send("marker", {event:"START"});
el("markFinish").onclick = () => send("marker", {event:"FINISH"});
el("stop").onclick = () => send("stop");
el("save").onclick = () => send("observations", observed());
el("openFolder").onclick = () => {
  el("folderMessage").textContent = "";
  send("open-folder");
};
async function poll() {
  try {
    if (!token) token = (await (await fetch("/api/session")).json()).token;
    const r = await fetch("/api/status");
    if (!r.ok) throw new Error();
    display(await r.json());
  } catch { showError("アプリとの接続が途切れました。PowerShellの起動状態を確認してください。"); }
  setTimeout(poll, 600);
}
poll();
