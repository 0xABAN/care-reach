import * as THREE from 'three';
import jointSettings from '../../joint_settings.json';
import { createScene } from './scene.js';
import './style.css';

const app = document.querySelector('#app');
const icon = {
  drop: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><path d="M12 2.8C9.5 6.2 5.7 10.5 5.7 14.2a6.3 6.3 0 0 0 12.6 0c0-3.7-3.8-8-6.3-11.4Z"/><path d="M8.8 14.6c0 1.9 1.3 3.3 3.1 3.5" stroke-linecap="round"/></svg>',
  play: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><path d="m8 5 11 7-11 7V5Z" stroke-linejoin="round"/></svg>',
  target: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 1v3m0 16v3M1 12h3m16 0h3"/></svg>',
  check: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="m4.5 12 5 5 10-10" stroke-linecap="round" stroke-linejoin="round"/></svg>',
  pause: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M8 5v14M16 5v14" stroke-linecap="round"/></svg>',
  stop: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><rect x="5" y="5" width="14" height="14" rx="2"/></svg>',
  arrow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><path d="M4 12h15m-6-6 6 6-6 6" stroke-linecap="round" stroke-linejoin="round"/></svg>',
  orbit: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><circle cx="12" cy="12" r="3"/><path d="M3 13a9 9 0 0 1 15-7M21 11a9 9 0 0 1-15 7" stroke-linecap="round"/><path d="m16 6 3 .1-.1-3M8 18l-3-.1.1 3" stroke-linecap="round" stroke-linejoin="round"/></svg>',
};

app.innerHTML = `
  <div class="site-shell">
    <header class="topbar">
      <div class="brand"><div class="brand-symbol">${icon.drop}</div><div><strong>Third Hand</strong><span>ASSISTIVE ROBOTICS · CONCEPT LAB</span></div></div>
        <div class="header-right"><span class="header-pill"><i></i> LOCAL SIMULATION</span><span id="graphics-status" class="header-version">GRAPHICS LOADING</span></div>
    </header>

    <main class="workspace">
      <section class="scene-panel" aria-label="Interactive 3D water assistance simulation">
        <div id="scene"></div>
        <div id="loading-overlay" class="loading-overlay" role="status" aria-live="polite"><div class="loading-mark">${icon.drop}</div><strong>Loading the 3D scene</strong><span>Preparing the wheelchair user and robotic arm…</span></div>
        <div class="scene-vignette"></div>
        <div class="scene-top-left">
          <div class="eyebrow bright"><span class="live-dot"></span> WATER ASSISTANCE · LOCAL SIMULATION</div>
          <h1>Water, <em>within reach.</em></h1>
          <p>A wheelchair-mounted arm assists with a simple daily task.</p>
        </div>
        <div class="scene-top-right"><div class="scene-chip">${icon.orbit}<span>DRAG TO ORBIT · SCROLL TO ZOOM</span></div></div>
        <div id="target-label" class="target-label" hidden><span class="target-plus">${icon.target}</span><span><strong>LIDDED WATER CUP</strong><small>SIMULATED CAMERA TARGET</small></span></div>
        <div class="scene-lower">
          <div class="scene-progress"><div class="scene-progress-head"><span id="scene-stage">READY TO BEGIN</span><span id="progress-text">00 / 06</span></div><div class="track"><div id="progress-fill"></div></div></div>
          <div class="view-switch" role="group" aria-label="Camera view"><button class="view active" data-view="overview">Overview</button><button class="view" data-view="side">Side view</button><button class="view" data-view="user">User view</button></div>
        </div>
      </section>

      <aside class="console" aria-label="Simulation controls">
        <div class="console-scroll">
          <div class="intro"><div class="eyebrow">THE THIRD HAND EXPERIENCE</div><h2>A simple request.<br><span>A meaningful response.</span></h2><p>Explore how a wheelchair-mounted arm could help someone drink water with control at every step.</p></div>

          <div class="mode-card"><span class="mode-icon">${icon.drop}</span><div><strong>Water assistance</strong><small>Voice-inspired interaction · no mic required</small></div><span class="mode-number">01</span></div>

          <div class="section-label"><span>THE INTERACTION</span><span id="stage-number">STEP 0 OF 6</span></div>
          <div class="steps" id="steps">
            <div class="step" data-step="1"><span class="step-index">01</span><span class="step-copy"><strong>Request water</strong><small>Start with a simple command</small></span><span class="step-indicator"></span></div>
            <div class="step" data-step="2"><span class="step-index">02</span><span class="step-copy"><strong>Find & confirm</strong><small>Camera identifies the cup</small></span><span class="step-indicator"></span></div>
            <div class="step" data-step="3"><span class="step-index">03</span><span class="step-copy"><strong>Approach & grasp</strong><small>Small, staged movement</small></span><span class="step-indicator"></span></div>
            <div class="step" data-step="4"><span class="step-index">04</span><span class="step-copy"><strong>Bring into reach</strong><small>Cup stays upright</small></span><span class="step-indicator"></span></div>
            <div class="step" data-step="5"><span class="step-index">05</span><span class="step-copy"><strong>Pause to drink</strong><small>Stops before the face</small></span><span class="step-indicator"></span></div>
            <div class="step" data-step="6"><span class="step-index">06</span><span class="step-copy"><strong>Return the cup</strong><small>Responds to “I'm done”</small></span><span class="step-indicator"></span></div>
          </div>

          <div class="status-card" aria-live="polite"><div class="status-top"><span class="status-kicker"><i></i> LIVE SIMULATION</span><span id="status-time">00:00</span></div><strong id="status-title">Ready when you are</strong><p id="status-description">Start a guided demo, or type a water request below.</p><div class="status-mini"><span id="status-reach">Workspace: concept check</span><span id="status-payload">Payload: unverified</span></div></div>

          <div class="command-block"><label for="command-input">VOICE-INSPIRED COMMAND</label><div class="command-input-wrap"><span class="quote-mark">“</span><input id="command-input" type="text" autocomplete="off" placeholder="Hey Arm, I'd like some water" aria-label="Type a water request or I'm done" /><button id="send-command" aria-label="Send typed command">${icon.arrow}</button></div><div class="transcript"><span>LAST COMMAND</span><strong id="transcript-text">No command yet</strong></div></div>

          <div id="confirm-card" class="confirm-card" hidden><div class="confirm-card-head"><span>${icon.target}</span><div><strong>Is this the right cup?</strong><small>Confirm the simulated camera target.</small></div></div><button id="confirm-button" class="confirm-button">${icon.check} Confirm water cup</button></div>

          <div class="primary-actions"><button id="guided-button" class="primary-button">${icon.play}<span>Replay simulation</span></button><button id="request-button" class="secondary-button">Start manually</button><button id="done-button" class="confirm-button" hidden>I'm done · return cup</button></div>
          <div class="safety-actions"><button id="pause-button" disabled>${icon.pause}<span>Pause</span></button><button id="stop-button" disabled>${icon.stop}<span>Emergency stop</span></button><button id="reset-button">Reset scene</button></div>
          <p class="key-hint">SPACE pauses · ESC stops · mouse rotates the scene</p>

          <div class="hardware-note"><div class="hardware-note-title"><span class="info-icon">i</span> Grounded in the current repo</div><p>The visualization uses a small arm workspace and the repository's saved joint-bound data as reference. Physical cup payload, camera calibration, face clearance, and grasp reliability have <strong>not been verified</strong>. Nothing here controls the Pi or motors.</p><button id="limits-button">View joint-bound reference ${icon.arrow}</button><div id="limits-content" hidden></div></div>
        </div>
      </aside>
    </main>
    <footer class="footer"><span><i></i> SIMULATED EXPERIENCE · NO MOTOR CONNECTION</span><span>Built for exploration, not unsupervised use near a person.</span></footer>
  </div>`;

const $ = id => document.getElementById(id);
const sim = createScene($('scene'));
const stageNames = {
  idle: ['Ready when you are', 'Replay the guided scene, or start the manual version.', 0, 'READY TO BEGIN'],
  scan: ['Looking for the cup', 'The fixed camera searches the nearby table.', 2, 'SCANNING TABLE'],
  confirm: ['Cup found. Your choice.', 'The arm waits until you confirm the target.', 2, 'WAITING FOR CONFIRMATION'],
  approach: ['Planning a clear approach', 'The gripper moves above the cup before descending.', 3, 'APPROACHING CUP'],
  lower: ['Moving in slowly', 'The arm descends along a staged path.', 3, 'ALIGNING GRIPPER'],
  grip: ['Gently securing the cup', 'The gripper closes around the lightweight cup.', 3, 'GRASPING CUP'],
  lift: ['Lifting upright', 'The cup remains level as it leaves the table.', 4, 'LIFTING CUP'],
  travelA: ['Bringing water closer', 'The planned route stays above the table and chair.', 4, 'MOVING TO USER'],
  travelB: ['Approaching sip position', 'The cup stops short of the person’s face.', 4, 'APPROACHING SIP POSITION'],
  sip: ['Water is ready', 'The arm stops short of the face and holds the cup steady for drinking.', 5, 'PAUSED FOR SIPPING'],
  returnA: ['Returning the cup', 'The arm retraces its clear route.', 6, 'RETURNING CUP'],
  returnB: ['Moving above the table', 'The cup remains upright on the way back.', 6, 'ABOVE TABLE'],
  replace: ['Setting the cup down', 'The arm places the cup on the table.', 6, 'PLACING CUP'],
  release: ['Releasing the cup', 'The gripper opens once the cup is supported.', 6, 'RELEASING CUP'],
  retreat: ['Returning to rest', 'The arm moves clear of the table.', 6, 'RETURNING TO REST'],
  complete: ['Task complete', 'The cup is back on the table. You remain in control.', 6, 'TASK COMPLETE'],
  stopped: ['Movement stopped', 'The simulated arm holds its current pose. Reset to begin again.', 0, 'EMERGENCY STOP'],
};
const steps = Array.from(document.querySelectorAll('.step'));
const state = { stage: 'idle', elapsed: 0, runTime: 0, paused: false, guided: false, carrying: false, transition: null, grip: null, toastTimer: 0 };
const UP = new THREE.Vector3(0, .085, 0);
const holdCupAt = point => sim.setCupPosition(point.clone().sub(UP));
const smooth = t => t * t * (3 - 2 * t);

function setStage(stage) {
  state.stage = stage;
  state.elapsed = 0;
  state.transition = null;
  state.grip = null;
  if (stage === 'scan') sim.setView('overview');
  if (stage === 'approach') sim.setView('detail');
  if (stage === 'travelB') sim.setView('sip');
  if (stage === 'returnB') sim.setView('overview');
  updateUI();
}

function startMove(stage, from, to, duration, next, carry = state.carrying) {
  setStage(stage);
  state.transition = { from: from.clone(), to: to.clone(), duration, progress: 0, next, carry };
}

function stageStep() {
  return stageNames[state.stage][2];
}

function updateUI() {
  const [title, description, step, sceneName] = stageNames[state.stage];
  $('status-title').textContent = state.paused ? 'Simulation paused' : title;
  $('status-description').textContent = state.paused ? 'The cup and arm are holding position. Resume whenever you are ready.' : description;
  $('stage-number').textContent = `STEP ${step} OF 6`;
  $('scene-stage').textContent = state.paused ? 'PAUSED' : sceneName;
  $('progress-text').textContent = `${String(step).padStart(2, '0')} / 06`;
  $('progress-fill').style.width = `${step === 0 ? 0 : Math.min(100, (step / 6) * 100)}%`;
  steps.forEach((element, index) => {
    const number = index + 1;
    element.classList.toggle('done', number < step || state.stage === 'complete');
    element.classList.toggle('active', number === step && state.stage !== 'complete');
  });
  $('confirm-card').hidden = state.stage !== 'confirm';
  $('done-button').hidden = state.stage !== 'sip';
  $('target-label').hidden = !['confirm', 'scan'].includes(state.stage);
  $('pause-button').disabled = ['idle', 'confirm', 'sip', 'complete', 'stopped'].includes(state.stage);
  $('pause-button').innerHTML = `${state.paused ? icon.play : icon.pause}<span>${state.paused ? 'Resume' : 'Pause'}</span>`;
  $('stop-button').disabled = ['idle', 'complete', 'stopped'].includes(state.stage);
  $('request-button').disabled = !['idle', 'complete'].includes(state.stage);
  $('guided-button').disabled = !['idle', 'complete'].includes(state.stage);
  $('status-reach').textContent = ['idle', 'scan', 'confirm', 'stopped'].includes(state.stage) ? 'Workspace: concept check' : 'Workspace: waypoints within visual reach';
  $('status-payload').textContent = 'Payload: unverified for real arm';
  document.querySelector('.site-shell').classList.toggle('playing', !['idle', 'complete', 'stopped'].includes(state.stage));
}

function setTranscript(text, accepted) {
  $('transcript-text').textContent = text;
  $('transcript-text').classList.toggle('rejected', !accepted);
}

function sayWater(guided = false) {
  if (state.stage === 'complete') reset();
  if (state.stage !== 'idle') return;
  state.guided = guided;
  setTranscript('“Hey Arm, I’d like some water.”', true);
  sim.setDetection(false);
  sim.setPath(false);
  setStage('scan');
}

function startStory(guided = true) {
  if (state.stage === 'complete') reset();
  if (state.stage !== 'idle') return;
  state.runTime = 0;
  sayWater(guided);
}

function checkRoute() {
  const p = sim.points;
  const route = [p.hover, p.grip, p.lift, p.transit, p.sip];
  const reaches = route.every(point => sim.canReach(point));
  // Conservative visual-route checks around the table, chair and face. These are not a physical safety certificate.
  const clearsTable = route.every(point => point.x < .15 || point.x > .88 || point.y >= 1.10);
  const mouth = new THREE.Vector3(-.62, 1.27, .12);
  const avoidsFace = route.every(point => point.distanceTo(mouth) > .20);
  const strawEnd = p.sip.clone().sub(UP).add(new THREE.Vector3(-.14, .290, -.16));
  const strawStopsShort = strawEnd.distanceTo(mouth) >= .025;
  return reaches && clearsTable && avoidsFace && strawStopsShort;
}

function confirmTarget() {
  if (state.stage !== 'confirm' || state.paused) return;
  if (!checkRoute()) {
    $('status-title').textContent = 'Route outside visual clearance';
    $('status-description').textContent = 'The simulated path failed its workspace or clearance check. Reset before trying again.';
    return;
  }
  sim.setDetection(false);
  sim.setPath(true);
  startMove('approach', sim.points.rest, sim.points.hover, 1.65, 'lower', false);
}

function finishDrinking() {
  if (state.stage !== 'sip') return;
  setTranscript('“I’m done.”', true);
  startMove('returnA', sim.points.sip, sim.points.transit, 1.7, 'returnB', true);
}

function pauseOrResume() {
  if ($('pause-button').disabled) return;
  state.paused = !state.paused;
  updateUI();
}

function emergencyStop() {
  if ($('stop-button').disabled) return;
  state.paused = false;
  state.guided = false;
  state.transition = null;
  setStage('stopped');
}

function reset() {
  state.guided = false;
  state.paused = false;
  state.carrying = false;
  state.runTime = 0;
  $('status-time').textContent = '00:00';
  sim.poseArm(sim.points.rest);
  sim.setCupPosition(sim.getSourceCupPosition());
  sim.setGrip(0);
  sim.setDetection(false);
  sim.setPath(false);
  setTranscript('No command yet', true);
  setStage('idle');
}

function handleCommand(raw) {
  const text = raw.trim();
  if (!text) return;
  $('command-input').value = '';
  const normalized = text.toLowerCase().replace(/[’']/g, '').replace(/[^a-z ]/g, ' ').replace(/\s+/g, ' ').trim();
  if (normalized.includes('done') || normalized.includes('finished')) {
    if (state.stage === 'sip') finishDrinking();
    else setTranscript(`“${text}” · wait until water is ready`, false);
    return;
  }
  if (normalized.includes('water') && ['idle', 'complete'].includes(state.stage)) {
    sayWater(false);
    setTranscript(`“${text}”`, true);
    return;
  }
  setTranscript(`“${text}” · not a water request`, false);
}

function followOn(next) {
  const p = sim.points;
  switch (next) {
    case 'lower': startMove('lower', p.hover, p.grip, 1.2, 'grip', false); break;
    case 'grip': setStage('grip'); sim.setGrip(0); break;
    case 'lift': state.carrying = true; startMove('lift', p.grip, p.lift, 1.3, 'travelA', true); break;
    case 'travelA': startMove('travelA', p.lift, p.transit, 1.6, 'travelB', true); break;
    case 'travelB': startMove('travelB', p.transit, p.sip, 1.8, 'sip', true); break;
    case 'sip': setStage('sip'); sim.setPath(false); break;
    case 'returnB': startMove('returnB', p.transit, p.lift, 1.65, 'replace', true); break;
    case 'replace': startMove('replace', p.lift, p.grip, 1.2, 'release', true); break;
    case 'release': setStage('release'); break;
    case 'retreat': state.carrying = false; sim.setCupPosition(sim.getSourceCupPosition()); startMove('retreat', p.grip, p.rest, 1.65, 'complete', false); break;
    case 'complete': setStage('complete'); sim.setDetection(false); sim.setPath(false); break;
  }
}

function advance(dt) {
  if (state.paused || ['idle', 'stopped', 'complete'].includes(state.stage)) return;
  state.elapsed += dt;
  state.runTime += dt;
  if (state.stage === 'scan') {
    if (state.elapsed > .58) sim.setDetection(true);
    if (state.elapsed >= 1.35) setStage('confirm');
  } else if (state.stage === 'confirm') {
    if (state.guided && state.elapsed >= 1.45) confirmTarget();
  } else if (state.transition) {
    const t = state.transition;
    t.progress = Math.min(1, t.progress + dt / t.duration);
    const point = t.from.clone().lerp(t.to, smooth(t.progress));
    sim.poseArm(point);
    if (t.carry) holdCupAt(point);
    if (t.progress >= 1) followOn(t.next);
  } else if (state.stage === 'grip') {
    sim.setGrip(Math.min(1, state.elapsed / .72));
    if (state.elapsed >= .80) followOn('lift');
  } else if (state.stage === 'sip') {
    if (state.guided && state.elapsed >= 3.1) finishDrinking();
  } else if (state.stage === 'release') {
    sim.setGrip(Math.max(0, 1 - state.elapsed / .65));
    if (state.elapsed >= .7) followOn('retreat');
  }
  const seconds = Math.floor(state.runTime);
  $('status-time').textContent = `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

$('guided-button').addEventListener('click', () => startStory(true));
$('request-button').addEventListener('click', () => startStory(false));
$('done-button').addEventListener('click', finishDrinking);
$('confirm-button').addEventListener('click', confirmTarget);
$('pause-button').addEventListener('click', pauseOrResume);
$('stop-button').addEventListener('click', emergencyStop);
$('reset-button').addEventListener('click', reset);
$('send-command').addEventListener('click', () => handleCommand($('command-input').value));
$('command-input').addEventListener('keydown', event => { if (event.key === 'Enter') handleCommand(event.currentTarget.value); });
document.querySelectorAll('.view').forEach(button => button.addEventListener('click', () => {
  document.querySelectorAll('.view').forEach(b => b.classList.remove('active'));
  button.classList.add('active');
  sim.setView(button.dataset.view);
}));
document.addEventListener('keydown', event => {
  if (event.target instanceof HTMLInputElement) return;
  if (event.code === 'Space') { event.preventDefault(); pauseOrResume(); }
  if (event.code === 'Escape') emergencyStop();
});

const limits = Object.entries(jointSettings).map(([joint, bounds]) => {
  const names = { 1: 'Shoulder pan', 2: 'Shoulder lift', 3: 'Elbow flex', 4: 'Wrist flex', 5: 'Wrist roll', 6: 'Gripper' };
  return `<div><span>${joint} · ${names[joint]}</span><strong>${bounds.lower ?? 'unverified'} – ${bounds.upper ?? 'unverified'} ticks</strong></div>`;
}).join('');
$('limits-content').innerHTML = `<div class="limits-grid">${limits}</div><p>Saved soft bounds from <code>joint_settings.json</code>. Visual arm poses are illustrative and are not converted to hardware ticks. The current repo has no validated filled-cup payload rating.</p>`;
$('limits-button').addEventListener('click', () => {
  $('limits-content').hidden = !$('limits-content').hidden;
  $('limits-button').classList.toggle('expanded', !$('limits-content').hidden);
});

let last = performance.now();
function frame(now) {
  const dt = Math.min(.12, (now - last) / 1000);
  last = now;
  advance(dt);
  sim.render(dt);
  $('scene').dataset.toolError = String(sim.getToolError());
  if (!$('target-label').hidden) {
    const point = sim.cupScreenPosition();
    $('target-label').style.left = `${point.x}px`;
    $('target-label').style.top = `${point.y}px`;
    $('target-label').style.opacity = point.visible ? '1' : '0';
  }
  requestAnimationFrame(frame);
}
updateUI();
requestAnimationFrame(frame);
sim.ready.then(() => {
  const graphics = sim.graphicsInfo();
  const software = /swiftshader|software|llvmpipe/i.test(graphics.renderer);
  $('graphics-status').textContent = software ? 'SOFTWARE WEBGL · NO MOTOR CONNECTION' : 'GPU WEBGL · NO MOTOR CONNECTION';
  $('graphics-status').title = graphics.renderer;
  $('loading-overlay').hidden = true;
  setTimeout(() => { if (state.stage === 'idle') startStory(true); }, 1200);
}).catch(error => {
  console.error('3D scene failed to load', error);
  $('loading-overlay').querySelector('strong').textContent = 'The 3D scene could not load';
  $('loading-overlay').querySelector('span').textContent = 'Check that the local model files are present, then reload this page.';
  $('graphics-status').textContent = 'MODEL LOAD ERROR';
});
