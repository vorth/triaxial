// testSupport.js
// Graph-vertex positions for tests that need geometry without running any
// physics: set positions by hand, then seed placements with the same
// chord-ratio construction the app uses (seeding.js).

import { seedPlacementPositions } from "./seeding.js";

export class SeededPositions {
  constructor() {
    this.positions = new Map();
  }

  setPosition(id, x, y, z) {
    this.positions.set(id, { x, y, z });
  }

  seedPlacement(existingChain, midVerts, apexVerts) {
    const knownPts = existingChain.map((id) => this.positions.get(id));
    const { mid, apex } = seedPlacementPositions(knownPts, midVerts.length, apexVerts.length);
    midVerts.forEach((id, i) => this.positions.set(id, mid[i]));
    apexVerts.forEach((id, i) => this.positions.set(id, apex[i]));
  }
}
