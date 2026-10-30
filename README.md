# triaxial

A vanilla-JS, no-build-step Three.js app for designing weave graphs: polygonal
cells joined vertex-to-vertex with a triangle adjacent to every edge.

Open `index.html` directly, or serve the directory with any static file
server (e.g. `python3 -m http.server`). The app shell (menu, files,
undo/redo, autosave) is the [`<design-app>`](https://github.com/vorth/design-app)
web component, loaded from its hosted URL, so a network connection is needed.

## Tests

```
npm test
```

Runs the test suites (`*.test.js`) via Node's built-in test runner. Run
`npm install` once first: the strip physics tests need Rapier, the one dev
dependency. In the browser, Rapier is loaded from unpkg.
