# Contributing

Use Node 22 or 24. Keep the lab dependency-free unless a concrete requirement justifies a package. Describe the changed visible behavior or model assumption. Preserve frame conventions, singular-case limitations, global work/output limits and honest physical boundaries in docs/contract.md.

Before submitting, run npm ci --ignore-scripts, npm run check, npm run test:coverage, npm run demo -- --check and node scripts/generate-model.mjs --check. Add meaningful regressions for kinematics, swept geometry, schema or control state changes. Generated results/models must reproduce; explain any intentional change. Check failure states and input-edit invalidation in the browser, keyboard controls and text equivalents to the canvas.

Keep shared-source attribution and the original archive/history. A generated cover is concept art; measured model figures remain separate. No paid API requirement, automatic device connection or flashing belongs in this lab. New hardware/firmware needs an explicit contract, official toolchain verification and separate physical acceptance.
