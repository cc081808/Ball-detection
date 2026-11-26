'use strict';
const video = document.querySelector('#video');
const start = document.querySelector('#start');
const stop = document.querySelector('#stop');
const status = document.querySelector('#status');
const token = new URLSearchParams(location.hash.slice(1)).get('key');
const client = crypto.randomUUID();
const headers = {'Authorization': `Bearer ${token}`, 'X-Phone-ID': client};
const canvas = document.createElement('canvas');
const context = canvas.getContext('2d');
let stream, running = false, generation = 0, wakeLock, lastMediaTime = -1;
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));

function freshVideoFrame() {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('The camera stopped delivering frames.')), 1000);
    if ('requestVideoFrameCallback' in video) {
      video.requestVideoFrameCallback((now, metadata) => {
        clearTimeout(timer);
        if (metadata.mediaTime === lastMediaTime) reject(new Error('Camera frame is frozen.'));
        else { lastMediaTime = metadata.mediaTime; resolve(); }
      });
    } else {
      clearTimeout(timer);
      reject(new Error('Please use an up-to-date Safari or Chrome browser.'));
    }
  });
}

async function sendFrames(myGeneration) {
  let count = 0;
  while (running && generation === myGeneration) {
    // A laptop-issued request precedes capture. Its timestamp bounds all network
    // delay without trusting the phone clock or accepting buffered old images.
    const request = await fetch('/capture', {headers, cache:'no-store', signal:AbortSignal.timeout(2000)});
    if (!request.ok) throw new Error(request.status === 409 ? 'Another phone is paired. Close its page first.' : 'Pairing expired. Scan the new QR code.');
    const challenge = await request.json();
    await freshVideoFrame();
    if (!running || generation !== myGeneration || document.hidden) return;
    const scale = Math.min(640 / video.videoWidth, 480 / video.videoHeight, 1);
    canvas.width = Math.round(video.videoWidth * scale);
    canvas.height = Math.round(video.videoHeight * scale);
    context.drawImage(video, 0, 0, canvas.width, canvas.height);
    const jpeg = await new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', .8));
    if (!jpeg) throw new Error('Camera image could not be encoded.');
    const prefix = new ArrayBuffer(4);
    new DataView(prefix).setUint32(0, challenge.id, false);
    const reply = await fetch('/upload', {method:'POST', headers, body:new Blob([prefix,jpeg]), signal:AbortSignal.timeout(2000)});
    if (!reply.ok) throw new Error('Video arrived too late. Check Wi-Fi, then tap Start camera again.');
    status.textContent = `Connected · ${++count} frames sent · ${canvas.width} × ${canvas.height}`;
    await pause(25);
  }
}

function stopCamera(message = 'Camera stopped. Tap Start camera to reconnect.') {
  running = false; generation++;
  stream?.getTracks().forEach(track => track.stop());
  stream = null; video.srcObject = null;
  wakeLock?.release().catch(() => {}); wakeLock = null;
  start.disabled = false; stop.disabled = true; status.textContent = message;
}
start.onclick = async () => {
  if (!token) { status.textContent = 'Scan the complete pairing QR code on the laptop first.'; return; }
  start.disabled = true;
  const myGeneration = ++generation;
  try {
    const acquired = await navigator.mediaDevices.getUserMedia({audio:false, video:{facingMode:{ideal:'environment'}, width:{ideal:640}, height:{ideal:480}, frameRate:{ideal:20,max:30}}});
    if (generation !== myGeneration) { acquired.getTracks().forEach(track => track.stop()); return; }
    stream = acquired;
    video.srcObject = stream; await video.play();
    if (generation !== myGeneration) return;
    running = true; stop.disabled = false; lastMediaTime = -1;
    stream.getVideoTracks()[0].onended = () => { if (generation === myGeneration) stopCamera('Camera permission or connection ended.'); };
    try { wakeLock = await navigator.wakeLock?.request('screen'); } catch (_) {}
    status.textContent = 'Pairing with laptop…';
    await sendFrames(myGeneration);
  } catch (error) { if (generation === myGeneration) stopCamera(error.message); }
};
stop.onclick = () => stopCamera();
document.addEventListener('visibilitychange', () => { if (document.hidden) stopCamera('Page was hidden. Return here and restart the camera.'); });
