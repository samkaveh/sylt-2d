# Pinball Game Improvement Plan

Based on code review and screenshot analysis of the demo mode across all phases.

---

## 1. Visual & Rendering Improvements

### 1.1 Ball Trail / Motion Blur
- **Problem:** The ball has no visual motion trail, making it hard to track at high speed.
- **Fix:** Add a fading trail of ghost circles behind the ball (last N positions with decreasing alpha). Store a ring buffer of `Vec2` positions in `PlayState`.

### 1.2 Wall & Cabinet Rendering Upgrade
- **Problem:** Walls are plain dark rectangles with a thin stroke — they look flat and uninteresting.
- **Fix:** Render walls with a gradient or bevel effect (lighter top edge, darker bottom edge). Add subtle metallic sheen or texture via layered rects with varying alpha.

### 1.3 Bumper Hit Particle Burst
- **Problem:** Bumper flash is a color change only; no particle feedback.
- **Fix:** On bumper hit, spawn a short-lived burst of small circle particles radiating outward from the hit point. Particles fade and shrink over ~0.3s. Store in a `Vec<BumperParticle>` in `PlayState`.

### 1.4 Drain Zone Animation
- **Problem:** The red drain bar is static. No visual urgency when the ball approaches.
- **Fix:** Add a pulsing glow effect (oscillating alpha/scale) to the drain zone, intensifying when the ball is within a certain Y threshold. Optional: red warning flash on the entire bottom when ball is draining.

### 1.5 Ball Glow / Shadow
- **Problem:** The ball rendering is decent but could pop more.
- **Fix:** Add a subtle drop shadow (offset dark ellipse below the ball) to give depth. Enhance the outer glow ring when the ball is moving fast (scale glow proportional to `velocity.length()`).

### 1.6 Fluid Pool Visual Enhancement
- **Problem:** Fluid metaballs look good but the individual particles are visible as small circles; the metaball contour can be jagged.
- **Fix:** Increase `metaball_resolution` during play. Add a subtle animated wave/ripple effect to the metaball contour by slightly modulating particle positions with a time-based sine offset. Consider adding tiny bubble particles that float upward inside the pool.

### 1.7 Flipper Arc Visualization
- **Problem:** When flippers are activated, there's no visual arc showing the sweep.
- **Fix:** During active flipper movement, draw a faint arc line from rest angle to current angle, fading as the flipper completes its stroke. This gives satisfying visual feedback.

### 1.8 Chain Link Rendering
- **Problem:** Chain links are plain rectangles. Look mechanical rather than physical.
- **Fix:** Render each chain link as a small rounded rect or oval. Draw connection pins (small circles) between links. Add subtle color gradient along the chain length.

---

## 2. Gameplay & Mechanics Improvements

### 2.1 Target Discrete Hit Detection
- **Problem:** Targets award `score / 10` continuously based on proximity (`main.rs:1825`). This means targets score passively while the ball rolls near them, with no satisfying "hit" moment.
- **Fix:** Add a `hit_cooldown: f32` timer per target element. When the ball enters hit range and the cooldown is zero, award the full score once and set cooldown to ~0.5s. This makes each hit feel intentional and prevents score farming.

### 2.2 Bumper Re-hit Cooldown
- **Problem:** Bumpers can re-boost the ball every frame it's in range if `body.velocity.dot(dir) < boost`. This can cause the ball to "stick" to a bumper being repeatedly boosted.
- **Fix:** Add a per-bumper cooldown timer (e.g., 0.3s). After a boost, the cooldown prevents re-boosting until it expires.

### 2.3 Plunger Physics
- **Problem:** The plunger sets `body.velocity = Vec2::new(0.0, charge)` directly. The ball only launches if `x > 5.5 && y < 3.0`. This is fragile — if the ball rolls slightly out of the lane, it can't be launched.
- **Fix:** Instead of velocity injection, use a force impulse: `body.velocity.y += charge`. Relax the position check to be within the plunger lane area (x > 5.0). Consider adding a plunger visual that compresses/extends with charge.

### 2.4 Flipper Power & Feel
- **Problem:** Flippers use a proportional gain of 45.0 with max speed 34.0. The ball can feel "heavy" when hit by a flipper because the flipper angular velocity is limited.
- **Fix:** Increase `PROPORTIONAL_GAIN` to ~60.0 and `MAX_SPEED` to ~42.0 for snappier response. Alternatively, add a small velocity kick to the ball on flipper contact based on the flipper's angular velocity at the moment of contact.

### 2.5 Ball Speed Cap
- **Problem:** No speed cap. The ball can reach very high velocities after multiple bumper hits, making it impossible to track or flip.
- **Fix:** After each physics step, clamp `ball.velocity.length()` to a maximum (e.g., 60.0). This keeps the game playable.

### 2.6 Gravity Tuning
- **Problem:** Gravity is set to `(0.0, -18.0)` which is quite strong. Combined with the tall playfield, the ball descends very quickly, making it hard to react.
- **Fix:** Reduce gravity to `(0.0, -14.0)` or `(0.0, -12.0)` for a floatier, more classic pinball feel. Alternatively, increase flipper power to compensate.

### 2.7 Drain Delay / Ball Save
- **Problem:** The ball drains instantly at `y < -3.5` with no grace period.
- **Fix:** Add a 2-second "ball save" window at the start of each ball launch. During this window, if the ball drains, it respawns at the plunger lane instead of being lost. Display "BALL SAVED!" text briefly.

### 2.8 Combo / Multiplier System
- **Problem:** Scoring is flat — each element awards a fixed score. No incentive for skillful play.
- **Fix:** Add a combo multiplier that increases with each consecutive hit within a time window (e.g., 3 seconds). Display the current multiplier (x2, x3, x4...) prominently. Reset on drain or after timeout.

---

## 3. HUD & UI Improvements

### 3.1 Compact HUD Panel
- **Problem:** The right-side HUD panel (`play_hud`) is 220px wide, consuming ~25% of the window. It shows verbose control instructions that experienced players don't need.
- **Fix:** Reduce to ~160px. Move controls to a tooltip that appears on first play or on hover. Make the panel semi-transparent so the playfield shows through.

### 3.2 Score Display Enhancement
- **Problem:** Score is plain yellow text. No animation on score change.
- **Fix:** Add a brief scale-up animation when score changes (text grows 1.3x then shrinks back over 0.2s). Show the score increment as a floating "+100" text that rises and fades from the scoring element's position.

### 3.3 Plunger Charge Indicator
- **Problem:** The plunger charge bar is small and above the plunger body. Hard to see during play.
- **Fix:** Make the charge bar wider, add percentage text, and color-code it (green → yellow → red gradient). Consider also showing it as a vertical bar along the right edge of the plunger lane.

### 3.4 Game Over Screen
- **Problem:** Game over is just a text label in the HUD panel. No dramatic moment.
- **Fix:** Add a full-screen overlay with "GAME OVER" in large text, the final score, high score comparison, and a "Press R to Restart" prompt. Add a brief camera shake effect on the final drain.

### 3.5 Ball Counter Visualization
- **Problem:** "BALLS REMAINING: 3" is plain text.
- **Fix:** Show small ball icons (filled circles) instead of a number. Each lost ball loses its fill. More intuitive and visually appealing.

---

## 4. Editor Improvements

### 4.1 Undo/Redo System
- **Problem:** No undo. A misplaced click requires manual cleanup.
- **Fix:** Maintain a `Vec<Vec<BoardElement>>` history stack. Push state on each element add/delete/move. `Ctrl+Z` pops and restores. Limit to last 50 states.

### 4.2 Element Duplication
- **Problem:** To place multiple identical elements, you must configure each one individually.
- **Fix:** Add a "Duplicate" button when an element is selected, or `Ctrl+D` shortcut. Creates a copy offset by (1, 1) from the original.

### 4.3 Multi-Select & Group Move
- **Problem:** Can only select and move one element at a time.
- **Fix:** `Shift+Click` to add to selection. `Ctrl+A` to select all. Drag moves all selected elements together.

### 4.4 Element Snap Alignment Guides
- **Problem:** Grid snap helps, but no visual alignment guides to other elements.
- **Fix:** When dragging an element, show horizontal/vertical guide lines to the nearest edges/centers of other elements. Snap when within a threshold.

### 4.5 Preview During Placement
- **Problem:** The placement preview is a ghost element at 0.35 alpha. It's functional but could show the element's hit area more clearly.
- **Fix:** Add a thin dashed outline showing the exact collision boundary. For bumpers, show the boost radius ring. For flippers, show the full arc of motion.

### 4.6 Editor Keyboard Shortcuts
- **Problem:** All tool switching requires clicking the side panel buttons.
- **Fix:** Add number key shortcuts: `1`=Select, `2`=Wall, `3`=Bumper, etc. Show shortcut hints on the tool buttons.

### 4.7 Element Properties Panel Improvements
- **Problem:** Properties are shown below the tool buttons in a flat list. No context-sensitivity for the selected element.
- **Fix:** When an element is selected, show its properties in a dedicated section at the top of the panel with clear headers. Group related properties. Add color picker for element color.

---

## 5. Demo Mode Improvements

### 5.1 Smarter Flipper AI
- **Problem:** Demo flippers cycle in a fixed `(cyc % 2.0) < 0.7` pattern. This looks robotic and doesn't track the ball.
- **Fix:** Implement simple ball-tracking AI: activate the flipper closest to the ball's X position when the ball is within the flipper's Y range and falling (velocity.y < 0). This makes the demo look like a real player.

### 5.2 Demo Timing
- **Problem:** Some phases feel too fast (BuildBoard: 0.4s) while others too slow (BallInPlay: 8s).
- **Fix:** Extend BuildBoard to 1.5s so the viewer can see the board appear. Reduce BallInPlay to 5s. Add a fade transition between editor tool showcases (0.3s crossfade).

### 5.3 Demo Score Counter
- **Problem:** Score stays at 0 throughout the demo because the demo never properly triggers scoring (ball is launched but flippers don't keep it in play effectively).
- **Fix:** Seed the demo score with a starting value or have the demo AI track the ball well enough to actually score points. This makes the demo more impressive.

### 5.4 Demo Replay Loop
- **Problem:** Demo runs once and stops ("Demo complete"). 
- **Fix:** After completion, auto-restart the demo after a 3-second pause. This is useful for kiosk/trade-show display.

---

## 6. Audio (New Feature)

### 6.1 Sound Effects
- **Problem:** No audio at all. Pinball is defined by its soundscape.
- **Fix:** Add sound effect triggers at:
  - Bumper hit: short "ping" or "boing"
  - Flipper activate: mechanical "clack"
  - Ball launch: spring "thwack"
  - Target hit: electronic "ding"
  - Drain: descending "wah-wah"
  - Ball save: triumphant "whoosh"
  - Use `rodio` crate for audio playback. Load `.wav` or `.ogg` files from an `assets/` directory.

### 6.2 Background Music
- **Fix:** Add a looping ambient/electronic background track at low volume. Fade out on game over.

---

## 7. Physics & Stability Fixes

### 7.1 Flipper Jitter
- **Problem:** The flipper hinge joint can show micro-jitter because the proportional servo fights the joint constraint every frame.
- **Fix:** The current `drive_flipper` approach (setting both angular and linear velocity) is good but the `SETTLE_ANGLE` dead-zone at 0.015 rad (~0.86°) is too tight. Increase to 0.03 rad. Also ensure the joint `softness` at 0.005 isn't fighting the servo — consider raising to 0.01.

### 7.2 Ball Tunneling
- **Problem:** At high speeds, the ball can tunnel through thin walls (the 0.5-width plunger divider).
- **Fix:** Increase wall minimum width to 0.6. Enable CCD (continuous collision detection) in the physics engine if available, or add swept-circle checks in the step function for the ball.

### 7.3 Fluid Particle Stability
- **Problem:** Fluid particles are sensors that collide with each other. At high ball speeds, particles can scatter wildly.
- **Fix:** Cap the displacement force applied to particles (currently `* 45.0` at `main.rs:1771`). Reduce to `* 25.0`. Add a damping force that pulls displaced particles back toward their rest positions.

### 7.4 Chain Stability
- **Problem:** The chain with `end_mass: 16.0` can oscillate violently when hit by a fast ball, causing links to fly apart.
- **Fix:** Add position correction after each step: if any chain link is more than `link_height * 1.5` from its parent, clamp it back. Alternatively, increase joint damping on chain joints.

---

## 8. Code Quality & Architecture

### 8.1 Separate Game Logic from Rendering
- **Problem:** The `view()` function is ~430 lines and mixes rendering with game state checks.
- **Fix:** Extract rendering into separate functions: `render_ball()`, `render_flippers()`, `render_bumpers()`, `render_fluid()`, `render_walls()`, `render_hud()`. Each takes only the relevant state as parameters.

### 8.2 Component System for Elements
- **Problem:** Element behavior (scoring, physics, rendering) is scattered across `detect_scoring()`, `build_element_bodies()`, `draw_element_shape()`, etc. Adding a new element type requires editing 5+ functions.
- **Fix:** Add trait methods to `ElementKind` or create a `BoardElement` trait with `fn create_bodies()`, `fn on_ball_hit()`, `fn render_editor()`, `fn render_play()`. This makes adding new element types self-contained.

### 8.3 Reduce `.clone()` Usage
- **Problem:** `model.elements.clone()` is called in `detect_scoring()`, `apply_fluid_drag()`, and `build_element_bodies()` every frame. Each clone allocates a new `Vec` and clones all `ElementKind` enums.
- **Fix:** Use references or iterators instead. For `detect_scoring`, iterate `model.elements.iter()` directly. The clones were likely needed to avoid borrow conflicts — restructure to use indices or temporary ID collections.

### 8.4 Event System
- **Problem:** Scoring, visual effects, and game state changes are tightly coupled in `detect_scoring()`.
- **Fix:** Create an event queue: `Vec<GameEvent>` where `GameEvent` is an enum (`BumperHit { position, score }`, `TargetHit { score }`, `BallDrained`, `BallSave`, etc.). Process events in a central `handle_events()` function that updates score, spawns particles, plays sounds, etc.

### 8.5 Configuration Constants
- **Problem:** Physics constants (gravity, flipper gain, ball radius, etc.) are scattered as top-level `const` and inline literals.
- **Fix:** Centralize into a `GameConfig` struct with all tunable parameters. Load from a TOML/JSON file or expose in the editor's settings panel for easy tuning.

---

## 9. Priority Ranking

| Priority | Item | Impact | Effort |
|----------|------|--------|--------|
| P0 | Smart flipper AI for demo | High | Medium |
| P0 | Ball trail / motion blur | High | Low |
| P0 | Target discrete hit detection | High | Low |
| P0 | Bumper re-hit cooldown | High | Low |
| P1 | Combo/multiplier system | High | Medium |
| P1 | Game over overlay screen | Medium | Low |
| P1 | Compact HUD panel | Medium | Low |
| P1 | Ball speed cap | Medium | Low |
| P1 | Flipper power tuning | Medium | Low |
| P1 | Undo/redo in editor | High | Medium |
| P2 | Bumper hit particles | Medium | Medium |
| P2 | Drain zone pulse animation | Low | Low |
| P2 | Sound effects | High | High |
| P2 | Demo replay loop | Low | Low |
| P2 | Element duplication | Medium | Low |
| P2 | Ball save mechanic | Medium | Low |
| P3 | Wall rendering upgrade | Low | Medium |
| P3 | Fluid pool enhancement | Low | Medium |
| P3 | Chain rendering upgrade | Low | Low |
| P3 | Flipper arc visualization | Low | Low |
| P3 | Multi-select in editor | Medium | High |
| P3 | Editor keyboard shortcuts | Low | Low |
| P3 | Code architecture refactoring | Medium | High |
