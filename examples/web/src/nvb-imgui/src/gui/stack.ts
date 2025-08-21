import { REND, N, BBox, MBBox, Widget, InputState, GlobalStyle } from "./gui.ts";
import { Layout } from "./layout_widgets/layout.ts";
import { Window } from "./layout_widgets/window.ts";
import { Grid } from "./layout_widgets/grid.ts";
import { WindowHeader } from "./layout_widgets/window.ts";

export class Stack<ActionType> { // root
  bbox: BBox;
  widgets: Widget<ActionType>[]; // is actually layouts

  constructor() {
    this.bbox = { left: 0, right: 0, top: 0, bottom: 0 };
    this.widgets = [];
  }

  render(c: REND): void {
    c.lineWidth = 1;
  }

  stack_render(c: REND, input_state: InputState) {
    for (let o = input_state.window_order.length - 1; o >= 0; o--) {
      const i = input_state.window_order[o];
      if (!input_state.window_active[i])
        continue;
      this.widgets[i].render(c);
    }
  }

  makeWindow(c: REND, input_state: InputState,
    action_config: {
      window: ActionType,
      header: ActionType,
      resizeable: ActionType,
      close_btn: ActionType,
    },
    config: {
      title?: string,
      x?: number,
      y?: number,
      width?: number,
      height?: number,
      min_width?: number,
      min_height?: number
    }
  ): Window<ActionType> {
    const idx = this.widgets.length;

    if (input_state.window_offsets.length <= this.widgets.length) {
      input_state.window_offsets.push({ x: 0, y: 0 });
      input_state.window_positions.push({ x: config.x ?? 0, y: config.y ?? 0 });
      input_state.window_sizes.push({ width: config.width ?? 200, height: config.height ?? 200 });
      input_state.window_active.push(true);
      input_state.window_minimised.push(false);
      input_state.window_order.unshift(idx);
    }

    const wind = new Window<ActionType>(c, action_config.window, action_config.header, action_config.resizeable, action_config.close_btn, [idx], input_state.window_positions[idx].x, input_state.window_positions[idx].y, input_state.window_sizes[idx].width, input_state.window_sizes[idx].height, config.title ?? "Hello, World!", { width: config.min_width ?? 50, height: config.min_height ?? 20 });
    this.widgets.push(wind);

    return wind;
  }

  static makeGrid<ActionType>(layout: Layout<ActionType>, action_type: ActionType, cols: number, relative_width: number): Grid<ActionType> {
    const grid = new Grid<ActionType>(action_type, layout.loc.concat([layout.widgets.length]), layout.cursor.x, layout.cursor.y + GlobalStyle.layout_commons.padding, relative_width * (layout.bbox.right - layout.bbox.left - 2 * GlobalStyle.layout_commons.padding), cols);
    layout.pushWidget(grid);
    return grid;
  }

  endEditing(input_state: InputState): void {
    for (let i = 0; i < this.widgets.length; i++) {
      const widget = this.widgets[i];
      if (!input_state.window_active[i])
        continue;
      if (input_state.window_minimised[i] && widget instanceof Window) {
        widget.bbox.bottom = widget.bbox.top + widget.header_height;
        for (let idx = 0; idx <= 2; idx++) {
          if (widget.widgets[idx] instanceof WindowHeader) {
            (widget.widgets[idx] as WindowHeader<ActionType>).window_state = "minimised";
            break;
          }
        }
      }
    }
  }

  requestAction(input_state: InputState): { wants_focus: boolean, action: N<ActionType> } {
    this.endEditing(input_state);

    for (let o = 0; o < input_state.window_order.length; o++) {
      const i = input_state.window_order[o];
      const widget = this.widgets[i];

      if (input_state.mouse_frame.released) {
        input_state.resizing_window = false;
        document.body.style.cursor = "default";
      }

      if (!input_state.window_active[i])
        continue;

      const ret = widget instanceof Window ? widget.requestWindowAction(input_state.window_minimised[i], input_state) : widget.requestAction(input_state);

      if (ret.wants_focus) {
        input_state.window_order.splice(o, 1);
        input_state.window_order.unshift(i);
      }

      const res = ret as { minimise: boolean, close: boolean, resize: boolean, iters: N<number>, wants_focus: boolean, action: ActionType }
      if (widget instanceof Window && res.iters != null) {
        if (res.resize) {
          input_state.resizing_window = true;
          document.body.style.cursor = "nwse-resize";
        }
        if (input_state.resizing_window && JSON.stringify(input_state.active_widget_loc) == JSON.stringify(widget.widgets[res.iters].loc)) {
          input_state.window_sizes[i].width += input_state.window_sizes[i].width < widget.min_size.width && input_state.mouse_delta_pos.x < 0 ? 0 : input_state.mouse_delta_pos.x;
          input_state.window_sizes[i].height += input_state.window_sizes[i].height < widget.min_size.height && input_state.mouse_delta_pos.y < 0 ? 0 : input_state.mouse_delta_pos.y;
          break;
        }
        if (res.close) {
          input_state.window_active[i] = false;
        }
        if (res.minimise) {
          input_state.window_minimised[i] = !input_state.window_minimised[i];
        }
        if (input_state.window_minimised[i])
          widget.bbox.bottom = widget.bbox.top + MBBox.calcHeight(widget.widgets[res.iters].bbox);
      }


      document.body.style.cursor = "default";

      if (input_state.moving_window && JSON.stringify(input_state.active_widget_loc) === JSON.stringify(widget.loc)) {
        if (input_state.mouse_frame.clicked) {
          input_state.window_order.splice(o, 1);
          input_state.window_order.unshift(i);
          input_state.window_offsets[i].x = widget.bbox.left - input_state.mouse_position.x;
          input_state.window_offsets[i].y = widget.bbox.top - input_state.mouse_position.y;
        } else if (input_state.mouse_down) {
          input_state.window_positions[i].x = input_state.window_offsets[i].x + input_state.mouse_position.x;
          input_state.window_positions[i].y = input_state.window_offsets[i].y + input_state.mouse_position.y;
        }
        break;
      }

      if (ret.action == null && (!MBBox.isInside(widget.bbox, input_state.mouse_position.x, input_state.mouse_position.y) || input_state.moving_window))
        continue;

      if (input_state.mouse_frame.released) {
        input_state.active_widget_loc = [];
      }

      return ret;
    }


    return { wants_focus: false, action: null };
  }
}
