# triaxial

A vanilla-JS, no-build-step Three.js app for designing weave graphs: polygonal
cells joined vertex-to-vertex with a triangle adjacent to every edge.

Open `index.html` directly, or serve the directory with any static file
server (e.g. `python3 -m http.server`).

## Tests

```
npm test
```

Runs the topology/physics test suite (`weaveGraph.test.js`) via Node's
built-in test runner - no dependencies required.
