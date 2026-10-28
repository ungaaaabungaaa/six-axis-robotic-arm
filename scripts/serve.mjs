// SPDX-License-Identifier: Apache-2.0
import http from 'node:http';
import { readFile } from 'node:fs/promises';
const args = process.argv.slice(2);
if (args.length && !(args.length === 2 && args[0] === '--port'))
  throw Error('Usage: npm start -- --port 8802');
const port = args.length ? Number(args[1]) : 8802;
if (!Number.isInteger(port) || port < 1024 || port > 65535)
  throw Error('Port must be 1024..65535.');
const files = new Map([
  ['/', ['web/index.html', 'text/html']],
  ...['style.css', 'app.mjs', 'worker.mjs'].map((n) => [
    '/' + n,
    ['web/' + n, n.endsWith('css') ? 'text/css' : 'text/javascript'],
  ]),
  ...['math.mjs', 'kinematics.mjs', 'motion.mjs', 'controller.mjs', 'scenes.mjs', 'lab.mjs', 'diagnostics.mjs'].map(
    (n) => ['/src/' + n, ['src/' + n, 'text/javascript']],
  ),
]);
http
  .createServer(async (req, res) => {
    const headers = {
      'Content-Security-Policy':
        "default-src 'none'; script-src 'self'; style-src 'self'; worker-src 'self'; connect-src 'none'; img-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'none'",
      'X-Content-Type-Options': 'nosniff',
      'Referrer-Policy': 'no-referrer',
      'Cross-Origin-Resource-Policy': 'same-origin',
      'Cache-Control': 'no-store',
    };
    if (![`127.0.0.1:${port}`, `localhost:${port}`].includes(req.headers.host)) {
      res.writeHead(403, headers).end();
      return;
    }
    if (!['GET', 'HEAD'].includes(req.method)) {
      res.writeHead(405, { ...headers, Allow: 'GET, HEAD' }).end();
      return;
    }
    const file = files.get(req.url);
    if (!file) {
      res.writeHead(404, headers).end();
      return;
    }
    try {
      const data = await readFile(new URL('../' + file[0], import.meta.url));
      res.writeHead(200, {
        ...headers,
        'Content-Type': file[1] + '; charset=utf-8',
        'Content-Length': data.length,
      });
      res.end(req.method === 'HEAD' ? undefined : data);
    } catch {
      res.writeHead(500, headers).end();
    }
  })
  .listen(port, '127.0.0.1', () =>
    console.log(`Six-axis lab: http://127.0.0.1:${port}/ · loopback only · no device access`),
  );
