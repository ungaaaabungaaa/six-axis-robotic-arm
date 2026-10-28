import test from 'node:test';
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import http from 'node:http';
import { once } from 'node:events';
test('real loopback server serves the complete module graph and rejects nonpublic/write/foreign-host requests', async () => {
  const port = 20000 + (process.pid % 10000),
    child = spawn(process.execPath, ['scripts/serve.mjs', '--port', String(port)], {
      cwd: new URL('../', import.meta.url),
      stdio: ['ignore', 'pipe', 'pipe'],
    });
  let stderr = '';
  child.stderr.on('data', (x) => (stderr += x));
  const request = (pathname, method = 'GET', host = `127.0.0.1:${port}`) =>
    new Promise((resolve, reject) => {
      const req = http.request(
        { host: '127.0.0.1', port, path: pathname, method, headers: { Host: host } },
        (res) => {
          let text = '';
          res.setEncoding('utf8');
          res.on('data', (x) => (text += x));
          res.on('end', () => resolve({ status: res.statusCode, headers: res.headers, text }));
        },
      );
      req.on('error', reject);
      req.end();
    });
  try {
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(Error('Server did not start: ' + stderr)), 5000);
      let text = '';
      child.once('error', (e) => {
        clearTimeout(timer);
        reject(e);
      });
      child.once('exit', (code) => {
        clearTimeout(timer);
        reject(Error('Server exited ' + code + stderr));
      });
      child.stdout.on('data', (x) => {
        text += x;
        if (text.includes('Six-axis lab:')) {
          clearTimeout(timer);
          resolve();
        }
      });
    });
    const page = await request('/');
    assert.equal(page.status, 200);
    assert.match(page.headers['content-security-policy'], /connect-src 'none'/);
    assert.equal(page.headers['x-content-type-options'], 'nosniff');
    const pending = ['/app.mjs', '/worker.mjs'],
      seen = new Set();
    while (pending.length) {
      const url = pending.pop();
      if (seen.has(url)) continue;
      seen.add(url);
      assert.ok(seen.size <= 20);
      const file = await request(url);
      assert.equal(file.status, 200, url);
      for (const match of file.text.matchAll(/(?:from\s*|import\s*)['"]([^'"]+)['"]/g)) {
        const specifier = match[1];
        assert.ok(specifier.startsWith('.'));
        pending.push(new URL(specifier, 'http://127.0.0.1:' + port + url).pathname);
      }
    }
    assert.equal(seen.size, 9);
    for (const url of ['/package.json', '/../LICENSE', '/%2e%2e/LICENSE', '/?x=1'])
      assert.equal((await request(url)).status, 404);
    assert.equal((await request('/', 'HEAD')).text, '');
    assert.equal((await request('/', 'POST')).status, 405);
    assert.equal((await request('/', 'GET', 'foreign.example')).status, 403);
  } finally {
    child.kill();
    if (child.exitCode === null) await once(child, 'exit');
  }
});
