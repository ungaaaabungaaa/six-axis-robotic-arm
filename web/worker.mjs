// SPDX-License-Identifier: Apache-2.0
import { runLab } from './src/lab.mjs';
self.onmessage = ({ data }) => {
  try {
    self.postMessage({ report: runLab(data) });
  } catch (error) {
    self.postMessage({ error: error.message });
  }
};
