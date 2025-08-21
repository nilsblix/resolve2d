import { REND, WidgetLoc, BBox, MBBox, MColor, Cursor, Widget, GlobalStyle, InputState, N } from "../gui.ts";
import { Layout } from "./layout.ts";
import { Draggable } from "../basic_widgets/draggable.ts";
import { ButtonState, Button } from "../basic_widgets/button.ts";

class Header<ActionType> implements Widget<ActionType> {
  bbox: BBox;
  action_type: ActionType;
  loc: WidgetLoc;
  widgets: Widget<ActionType>[];
  title: string;
  title_font_size: number;
  window_state: "open" | "minimised";

  constructor(loc: WidgetLoc, action_type: ActionType, title: string, x: number, y: number, width: number) {
    this.action_type = action_type;
    this.title = title;
    this.title_font_size = GlobalStyle.header_commons.font_size;
    this.bbox = {
      left: x,
      top: y,
      right: x + width,
      bottom: y + 1.4 * GlobalStyle.header_commons.font_size,
    }
    this.loc = loc;
    this.widgets = [];
    this.window_state = "open";
  }

  render(c: REND) {
    c.fillStyle = this.window_state == "open" ? GlobalStyle.header_commons.bg_color : GlobalStyle.window.minimized_header_bg;
    c.fillRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));

    c.font = this.title_font_size + "px " + GlobalStyle.font;
    c.fillStyle = GlobalStyle.header_commons.color;
    c.textBaseline = "middle";
    c.textAlign = "left";

    const x = this.bbox.left + GlobalStyle.layout_commons.widget_gap;
    const y = (this.bbox.top + this.bbox.bottom) / 2;
    c.fillText(this.title, x, y);
  }

  requestAction(input_state: InputState): { wants_focus: boolean, action: N<ActionType> } {
    if (this.action_type == null)
      return { wants_focus: false, action: null };

    const [x, y] = [input_state.mouse_position.x, input_state.mouse_position.y];

    const inside = MBBox.isInside(this.bbox, x, y);
    if (inside && input_state.mouse_frame.double_clicked)
      return { wants_focus: false, action: this.action_type };

    return { wants_focus: false, action: null };
  }

}

class Resizeable<ActionType> extends Draggable<ActionType> implements Widget<ActionType> {
  constructor(c: REND, action_type: ActionType, loc: WidgetLoc, cursor: Cursor, text: string) {
    super(c, action_type, loc, cursor, text);
  }
}

class CloseButton<ActionType> extends Button<ActionType> implements Widget<ActionType> {
  constructor(c: REND, action_type: ActionType, loc: WidgetLoc, cursor: Cursor) {
    super(c, action_type, loc, cursor, "x");
  }

  render(c: REND): void {
    let color = GlobalStyle.widget.default_bg_color;
    if (this.state == ButtonState.hovered)
      color = GlobalStyle.widget.hover_bg_color;
    if (this.state == ButtonState.clicked)
      color = GlobalStyle.widget.down_bg_color;
    if (this.state == ButtonState.down)
      color = GlobalStyle.widget.down_bg_color;
    if (this.state == ButtonState.released)
      color = GlobalStyle.widget.down_bg_color;

    const x = (this.bbox.left + this.bbox.right) / 2;
    let y = (this.bbox.top + this.bbox.bottom) / 2;
    y += GlobalStyle.button.font_size * 0.1;

    c.fillStyle = color;
    c.beginPath();
    c.arc(x, y, 0.5 * GlobalStyle.button.font_size, 0, 2 * Math.PI);
    c.fill();
    c.closePath();

    c.font = `normal ${1.25 * GlobalStyle.button.font_size}px ${GlobalStyle.font}`;
    c.fillStyle = MColor.string(MColor.white);
    c.textBaseline = "middle";
    c.textAlign = "center";

    c.fillText(this.text, x, y);
  }
}

export class Window<ActionType> extends Layout<ActionType> implements Widget<ActionType> {
  min_size: { width: number, height: number };
  header_height: number;
  offset: number;
  mode: "normal" | "two columns";
  tools: {
    min_width: number,
    max_width: number,
  }

  constructor(c: REND, window_action_type: ActionType, header_action_type: ActionType, resizeable_action_type: ActionType, close_btn_action_type: ActionType, loc: WidgetLoc, x: number, y: number, width: number, height: number, title: string, min_size: { width: number, height: number }) {
    super(window_action_type, loc, x, y, width, height);
    this.offset = 0;
    this.mode = "normal";
    this.tools = {
      min_width: 0,
      max_width: MBBox.calcWidth(this.bbox),
    };

    const resize_loc = this.loc.concat([]);
    resize_loc.push(this.widgets.length);

    if (resizeable_action_type != null)
      this.widgets.push(new Resizeable<ActionType>(c, resizeable_action_type, resize_loc, { x: this.bbox.right - 27, y: this.bbox.bottom - 20 }, "res"));

    const close_loc = this.loc.concat([]);
    close_loc.push(this.widgets.length);

    if (close_btn_action_type != null)
      this.widgets.push(new CloseButton<ActionType>(c, close_btn_action_type, close_loc, { x: this.bbox.right - 15, y: this.bbox.top - 2.5 }));

    const header_loc = loc.concat([]);
    header_loc.push(this.widgets.length);

    const header = new Header(header_loc, header_action_type, title, this.cursor.x, this.cursor.y, 10);
    this.widgets.push(header);
    this.cursor.y += MBBox.calcHeight(header.bbox) + GlobalStyle.layout_commons.widget_gap;
    this.header_height = MBBox.calcHeight(header.bbox);

    this.min_size = min_size;

    header.bbox.left = this.bbox.left;
    header.bbox.right = this.bbox.right;
  }

  updateBBox(): void { }

  setMode(mode: "normal" | "two columns", config?: { min_width: number, max_width: number }): void {
    this.mode = mode;
    switch (this.mode) {
      case "normal":
        break;
      case "two columns":
        this.offset = this.widgets.length;
        if (config != undefined) {
          this.tools.min_width = config.min_width != undefined ? config.min_width : this.tools.min_width;
          this.tools.max_width = config.max_width != undefined ? config.max_width : this.tools.max_width;
        }
        break;
    }
  }

  pushWidget(widget: Widget<ActionType>): void {
    super.pushWidget(widget);

    switch (this.mode) {
      case "normal":
        this.cursor.y += MBBox.calcHeight(widget.bbox) + GlobalStyle.layout_commons.widget_gap;
        break;
      case "two columns":
        if ((this.widgets.length - this.offset) % 2 == 0) {
          this.cursor.x = this.bbox.left + GlobalStyle.layout_commons.padding;
          const last_idx = this.widgets.length - 1;
          const row_height = Math.max(MBBox.calcHeight(this.widgets[last_idx].bbox), MBBox.calcHeight(this.widgets[last_idx - 1].bbox));
          this.cursor.y += row_height + GlobalStyle.layout_commons.widget_gap;
        } else {
          this.cursor.x += Math.min(this.tools.max_width, Math.max(this.tools.min_width, MBBox.calcWidth(this.bbox))) / 2;
        }
        break;
    }
  }

  render(c: REND): void {
    c.save();

    const p = GlobalStyle.layout_commons.border_width;
    c.beginPath();
    c.rect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));
    c.closePath();

    c.fillStyle = MColor.string(GlobalStyle.layout_commons.bg_color);
    c.fillRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));

    c.strokeStyle = GlobalStyle.layout_commons.border;
    c.lineWidth = p;
    c.strokeRect(this.bbox.left - 0.5 * p, this.bbox.top - 0.5 * p, MBBox.calcWidth(this.bbox) + p, MBBox.calcHeight(this.bbox) + p);
    c.clip();
    super.render(c);

    c.restore();
  }

  requestAction(input_state: InputState): { minimise: boolean, close: boolean, resize: boolean, iters: N<number>, wants_focus: boolean; action: N<ActionType>; } {
    input_state.mouse_position.x;
    return { minimise: false, close: false, resize: false, iters: null, wants_focus: false, action: null };
  }

  requestWindowAction(minimised: boolean, input_state: InputState): { minimise: boolean, close: boolean, resize: boolean, iters: N<number>, wants_focus: boolean; action: N<ActionType>; } {
    const ret = super.requestAction(input_state);

    if (ret.iters == null) // => no widget has been found
      return { minimise: false, close: false, resize: false, ...ret }

    // from here on out a widget wants to do something
    // if part of a standard window flag ==> do some standard window action (close, mini etc)
    // else if not mini ==> just return the action


    if (this.widgets[ret.iters] instanceof Header)
      return { minimise: ret.action != null, close: false, resize: false, ...ret };
    if (this.widgets[ret.iters] instanceof Resizeable && !minimised)
      return { minimise: false, close: false, resize: ret.action != null, ...ret };
    if (this.widgets[ret.iters] instanceof CloseButton)
      return { minimise: false, close: ret.action != null, resize: false, ...ret };
    else if (!minimised)
      return { minimise: false, close: false, resize: false, ...ret };

    // window is minimised ==> this cannot return anything
    return { minimise: false, close: false, resize: false, iters: null, wants_focus: false, action: null };
  }

}

export { Resizeable as WindowResizeable };
export { CloseButton as WindowCloseButton };
export { Header as WindowHeader };
