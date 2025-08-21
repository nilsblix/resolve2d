import * as gui from "./gui/gui.ts";

// USER BASED THING.
// NOT PART OF GUI FRAMEWORK // USER

enum UIAction {
  placeholder,
  increment,
  decrement,
  add_new_window,
  drag_num,
  drag_text_wrap_width,
  HIGH_drag_text_wrap_width,
  bg_color_r, bg_color_g, bg_color_b,
  toggle_window_1,
  change_bg_color_with_picker, change_bg_color_real_width_picker,
  change_bg_color_hue, change_bg_color_saturation, change_bg_color_brightness,
  change_bg_color_r, change_bg_color_g, change_bg_color_b,
}

//const c = <gui.REND>gui.canvas.getContext("2d");
//const input_state = new gui.InputState(gui.canvas, 0, 0);

let num = 0;
let dt = -1;
let last_time = 0;
let frame_time = -1;
let num_windows = 1;

let make_time = -1;
let action_time = -1;
let render_time = -1;

let text_wrap_width = 400;

let bg_color: gui.Color = gui.MColor.fromHex("#738C99");

function update() {
  document.body.style.backgroundColor = gui.MColor.string(bg_color);

  const st = performance.now();

  gui.updateCanvasSizing();

  const stack = new gui.Stack<gui.N<UIAction>>();

  const mst = performance.now();

  const q = stack.makeWindow(gui.c, gui.input_state, { window: UIAction.placeholder, header: UIAction.placeholder, resizeable: UIAction.placeholder, close_btn: null }, { title: "debug info", width: 600, height: 500 });
  q.makeLabel(gui.c, null, "gui-time = " + dt.toFixed(2));
  q.makeLabel(gui.c, null, "frt = " + frame_time.toFixed(4));
  q.makeLabel(gui.c, null, "active loc = " + gui.input_state.active_widget_loc);
  q.makeLabel(gui.c, null, "resizing = " + gui.input_state.resizing_window);
  q.makeLabel(gui.c, null, "moving wind = " + gui.input_state.moving_window);
  q.makeLabel(gui.c, null, "make_dt = " + make_time.toFixed(4));
  q.makeLabel(gui.c, null, "action_dt = " + action_time.toFixed(4));
  q.makeLabel(gui.c, null, "render_dt = " + render_time.toFixed(4));
  q.makeLabel(gui.c, null, "active windows = " + gui.input_state.window_active);
  q.makeLabel(gui.c, null, "minim windows = " + gui.input_state.window_minimised);
  q.makeLabel(gui.c, null, "frame act = " + JSON.stringify(gui.input_state.mouse_frame));

  const picker_wind = stack.makeWindow(gui.c, gui.input_state, { window: UIAction.placeholder, header: UIAction.placeholder, resizeable: null, close_btn: null }, { title: "color picker test", width: 300, height: 380, x: 800, y: 100 });

  const picker_wind_usable_width = gui.MBBox.calcWidth(picker_wind.bbox) - 2 * gui.GlobalStyle.layout_commons.padding - 2 * gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.makeColorPickerRect(UIAction.change_bg_color_with_picker, gui.MColor.fromHex(gui.GlobalStyle.widget.default_bg_color), picker_wind_usable_width + 2 * gui.GlobalStyle.layout_commons.widget_gap, 300);

  const hsv_h = picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_hue, "H: " + Math.round(gui.MColor.toHSVA(bg_color).h), { width: 1 / 3 * picker_wind_usable_width });
  picker_wind.cursor.x = hsv_h.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.cursor.y = hsv_h.bbox.top;

  const hsv_s = picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_saturation, "S: " + gui.round(gui.MColor.toHSVA(bg_color).s, 2), { width: 1 / 3 * picker_wind_usable_width });
  picker_wind.cursor.x = hsv_s.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.cursor.y = hsv_s.bbox.top;

  const hsv_v = picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_brightness, "V: " + gui.round(gui.MColor.toHSVA(bg_color).v, 2), { width: 1 / 3 * picker_wind_usable_width });
  picker_wind.cursor.x = hsv_v.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.cursor.y = hsv_v.bbox.top;

  picker_wind.resetCursor();

  const rgb_r = picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_r, "R: " + Math.round(bg_color.r), { width: 1 / 3 * picker_wind_usable_width });
  picker_wind.cursor.x = rgb_r.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.cursor.y = rgb_r.bbox.top;

  const rgb_g = picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_g, "G: " + Math.round(bg_color.g), { width: 1 / 3 * picker_wind_usable_width });
  picker_wind.cursor.x = rgb_g.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
  picker_wind.cursor.y = rgb_g.bbox.top;

  picker_wind.makeDraggable(gui.c, UIAction.change_bg_color_b, "B: " + Math.round(bg_color.b), { width: 1 / 3 * picker_wind_usable_width });

  for (let i = 0; i < num_windows; i++) {
    if (i == 0) {
      const l = stack.makeWindow(gui.c, gui.input_state, { window: UIAction.placeholder, header: UIAction.placeholder, resizeable: UIAction.placeholder, close_btn: null }, { title: "window 1", x: 100, y: 100 });
      const l_usable_width = gui.MBBox.calcWidth(l.bbox) - 2 * gui.GlobalStyle.layout_commons.padding;
      l.makeButton(gui.c, UIAction.increment, "Increment");
      l.makeButton(gui.c, UIAction.decrement, "Decrement");
      const DRAG_SOME_NUMBER = l.makeLabel(gui.c, null, "Drag some arbitrary number:");
      l.cursor.y = DRAG_SOME_NUMBER.bbox.top;
      l.cursor.x = DRAG_SOME_NUMBER.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
      l.makeDraggable(gui.c, UIAction.drag_num, "number = " + num).bbox.bottom = DRAG_SOME_NUMBER.bbox.bottom;
      l.resetCursor();
      l.makeDraggable(gui.c, UIAction.drag_text_wrap_width, "Wrap width: " + text_wrap_width, { width: l_usable_width });
      l.makeLabel(gui.c, null, " ");
      l.makeButton(gui.c, UIAction.add_new_window, "Add a window. " + num_windows);
      l.makeText(gui.c, UIAction.increment, "Some more standard placeholder text that isn't some weird loremm ipsum shit that everyone is tired of. Wow have I offended someone with that statement. Fuck you those that are offended. I don't give a fuck", text_wrap_width);
      l.makeText(gui.c, UIAction.increment, "Some more standard placeholder text that isn't some weird loremm ipsum shit that everyone is tired of.", text_wrap_width);
      l.makeButton(gui.c, UIAction.toggle_window_1, "Toggle window 1");

      l.makeLabel(gui.c, null, " ");

      l.makeLabel(gui.c, null, "* Grid => ");
      const g = gui.Stack.makeGrid(l, UIAction.placeholder, 3, 1.0);
      g.makeDraggable(gui.c, UIAction.drag_text_wrap_width, "g00");
      g.makeDraggable(gui.c, UIAction.drag_num, "g10");
      g.makeDraggable(gui.c, UIAction.drag_num, "g20");
      g.makeDraggable(gui.c, UIAction.drag_num, "g01");
      g.makeDraggable(gui.c, UIAction.drag_num, "g11");
      g.makeDraggable(gui.c, UIAction.drag_num, "g21");
      g.makeDraggable(gui.c, UIAction.drag_num, "g02");
      g.makeDraggable(gui.c, UIAction.drag_num, "g12");
      g.makeDraggable(gui.c, UIAction.HIGH_drag_text_wrap_width, "g22");
      l.resetCursor();
      l.makeLabel(gui.c, null, "Testing text after grid");

      l.makeLabel(gui.c, null, " ");

      l.makeLabel(gui.c, null, "* bg color: ");
      const padd = " ";
      const color_draggable_width = 1 / 3 * (l_usable_width - 2 * gui.GlobalStyle.layout_commons.widget_gap)
      const bg1 = l.makeDraggable(gui.c, UIAction.bg_color_r, padd + "R: " + bg_color.r + padd, { width: color_draggable_width });
      l.cursor.x = bg1.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
      l.cursor.y = bg1.bbox.top;
      const bg2 = l.makeDraggable(gui.c, UIAction.bg_color_g, padd + "G: " + bg_color.g + padd, { width: color_draggable_width });
      l.cursor.x = bg2.bbox.right + gui.GlobalStyle.layout_commons.widget_gap;
      l.cursor.y = bg2.bbox.top;
      l.makeDraggable(gui.c, UIAction.bg_color_b, padd + "B: " + bg_color.b + padd, { width: color_draggable_width });
      l.resetCursor();
      l.makeLabel(gui.c, null, "testing after rgb")

    } else if (i == 1) {
      const l = stack.makeWindow(gui.c, gui.input_state, { window: UIAction.placeholder, header: UIAction.placeholder, resizeable: UIAction.placeholder, close_btn: UIAction.placeholder }, { title: "test window with different layout modes", x: 400, y: 100 });
      const [width] = [gui.MBBox.calcWidth(l.bbox) - gui.GlobalStyle.layout_commons.padding, gui.MBBox.calcHeight(l.bbox) - 2 * gui.GlobalStyle.layout_commons.padding];
      const col_width = Math.min(600, Math.max(300, width)) / 2;
      l.makeDraggable(gui.c, UIAction.change_bg_color_r, "Drag for bg r");
      l.makeLabel(gui.c, null, "Two column mode: ");
      l.setMode("two columns", { min_width: 300, max_width: 600 });
      l.makeLabel(gui.c, null, "some left side text");
      l.makeLabel(gui.c, null, "some right text");
      l.makeButton(gui.c, UIAction.increment, "Increment", { width: 100, height: 50 });
      l.makeButton(gui.c, UIAction.decrement, "Decrement");
      l.makeColorPickerRect(UIAction.change_bg_color_real_width_picker, bg_color, col_width, col_width);
      l.makeText(gui.c, null, "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.", col_width - 10);
    }
  }

  const met = performance.now();
  make_time = met - mst;

  const ast = performance.now();
  const ret = stack.requestAction(gui.input_state);
  const aet = performance.now();
  action_time = aet - ast;

  const action = ret.action;

  switch (action) {
    case UIAction.increment:
      num++;
      break;
    case UIAction.decrement:
      num--;
      break;
    case UIAction.add_new_window:
      num_windows++;
      break;
    case UIAction.drag_num:
      num += gui.input_state.mouse_delta_pos.x;
      if (num < -100) num = -100;
      if (num > 100) num = 100;
      break;
    case UIAction.drag_text_wrap_width:
      text_wrap_width += gui.input_state.mouse_delta_pos.x;
      if (text_wrap_width < 0) text_wrap_width = 0;
      break;
    case UIAction.HIGH_drag_text_wrap_width:
      text_wrap_width += 2 * gui.input_state.mouse_delta_pos.x;
      if (text_wrap_width < 0) text_wrap_width = 0;
      break;
    case UIAction.bg_color_r:
      bg_color.r += gui.input_state.mouse_delta_pos.x;
      break;
    case UIAction.bg_color_g:
      bg_color.g += gui.input_state.mouse_delta_pos.x;
      break;
    case UIAction.bg_color_b:
      bg_color.b += gui.input_state.mouse_delta_pos.x;
      break;
    case UIAction.toggle_window_1:
      gui.input_state.window_active[2] = !gui.input_state.window_active[2];
      break;
    case UIAction.change_bg_color_with_picker:
      if (gui.MColor.isColor(gui.input_state.action_ret_var))
        gui.GlobalStyle.widget.default_bg_color = gui.MColor.toHex(gui.input_state.action_ret_var);
      break;
    case UIAction.change_bg_color_hue:
      let { h: h1, s: s1, v: v1, a: a1 } = gui.MColor.toHSVA(bg_color);
      h1 = gui.updateDraggableValue(h1, gui.input_state, 1.0);
      bg_color = gui.MColor.fromHSVA(h1, s1, v1, a1);
      break;
    case UIAction.change_bg_color_saturation:
      let { h: h2, s: s2, v: v2, a: a2 } = gui.MColor.toHSVA(bg_color);
      s2 = gui.updateDraggableValue(s2, gui.input_state, 0.005, { min: 0, max: 1 });
      bg_color = gui.MColor.fromHSVA(h2, s2, v2, a2);
      break;
    case UIAction.change_bg_color_brightness:
      let { h: h3, s: s3, v: v3, a: a3 } = gui.MColor.toHSVA(bg_color);
      v3 = gui.updateDraggableValue(v3, gui.input_state, 0.005, { min: 0, max: 1 });
      bg_color = gui.MColor.fromHSVA(h3, s3, v3, a3);
      break;
    case UIAction.change_bg_color_r:
      bg_color.r = gui.updateDraggableValue(bg_color.r, gui.input_state, 1.0, { min: 0, max: 255 });
      break;
    case UIAction.change_bg_color_g:
      bg_color.g = gui.updateDraggableValue(bg_color.g, gui.input_state, 1.0, { min: 0, max: 255 });
      break;
    case UIAction.change_bg_color_b:
      bg_color.b = gui.updateDraggableValue(bg_color.b, gui.input_state, 1.0, { min: 0, max: 255 });
      break;
    case UIAction.change_bg_color_real_width_picker:
      if (gui.MColor.isColor(gui.input_state.action_ret_var))
        bg_color = gui.input_state.action_ret_var;
  }

  const rst = performance.now();
  gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
  stack.stack_render(gui.c, gui.input_state);
  const rret = performance.now();
  render_time = rret - rst;

  gui.input_state.end();

  const et = performance.now();
  dt = et - st;
  frame_time = et - last_time;
  last_time = et;

  requestAnimationFrame(update)
}

update();
