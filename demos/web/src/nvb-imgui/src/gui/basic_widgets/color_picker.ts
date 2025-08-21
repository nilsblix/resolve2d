import { REND, N, WidgetLoc, Cursor, BBox, MBBox, Color, MColor, Widget, InputState } from "../gui.ts";

export class ColorPickerRect<ActionType> implements Widget<ActionType> {
  bbox: BBox;
  action_type: ActionType;
  loc: WidgetLoc;
  color: Color;

  constructor(action_type: ActionType, loc: WidgetLoc, cursor: Cursor, color: Color, width: number, height: number) {
    this.color = color;
    this.bbox = { left: cursor.x, top: cursor.y, right: cursor.x + width, bottom: cursor.y + height };
    this.action_type = action_type;
    this.loc = loc;
  }

  render(c: REND): void {
    const width = MBBox.calcWidth(this.bbox);
    const height = MBBox.calcHeight(this.bbox);

    const hsv = MColor.toHSVA(this.color);

    const hsv_full_sat = { h: hsv.h, s: hsv.s, v: hsv.v, a: hsv.a };
    hsv_full_sat.s = 1;
    hsv_full_sat.v = 1;

    const full_sat_string = MColor.string(MColor.fromHSVA(hsv_full_sat.h, hsv_full_sat.s, hsv_full_sat.v, hsv_full_sat.a));

    const sat_grad = c.createLinearGradient(this.bbox.left, this.bbox.top, this.bbox.right, this.bbox.top);
    sat_grad.addColorStop(0, "white");
    sat_grad.addColorStop(1, full_sat_string);
    c.fillStyle = sat_grad;
    c.fillRect(this.bbox.left, this.bbox.top, width, height);

    const bright_grad = c.createLinearGradient(this.bbox.left, this.bbox.top, this.bbox.left, this.bbox.bottom);
    bright_grad.addColorStop(0, 'rgba(0, 0, 0, 0)');
    bright_grad.addColorStop(1, 'rgba(0, 0, 0, 1)');
    c.fillStyle = bright_grad;
    c.fillRect(this.bbox.left, this.bbox.top, width, height);

    const x = hsv.s * width + this.bbox.left;
    const y = (1 - hsv.v) * height + this.bbox.top;

    c.beginPath();
    c.arc(x, y, 5, 0, 2 * Math.PI);
    c.fillStyle = MColor.string(this.color);
    c.fill();
    c.lineWidth = 3;
    c.strokeStyle = "black";
    c.stroke();
    c.lineWidth = 2;
    c.strokeStyle = "white";
    c.stroke();
    c.closePath();

  }

  /**
   * The color that is going to be changed will be stored inside input_state.action_ret_var
   *
  */
  requestAction(input_state: InputState): { wants_focus: boolean, action: N<ActionType> } {
    if (this.action_type == null)
      return { wants_focus: false, action: null };
    const [x, y] = [input_state.mouse_position.x, input_state.mouse_position.y];

    const inside = MBBox.isInside(this.bbox, x, y);

    if (inside && input_state.mouse_frame.clicked)
      input_state.active_widget_loc = this.loc;

    const same_loc = (JSON.stringify(input_state.active_widget_loc) == JSON.stringify(this.loc));

    if (same_loc && input_state.mouse_down) {
      const width = MBBox.calcWidth(this.bbox);
      const height = MBBox.calcHeight(this.bbox);

      const clamp01 = (v: number) => Math.min(Math.max(v, 0.0015), 0.9990);

      const sat_scalar = clamp01((x - this.bbox.left) / width);
      const bright_scalar = clamp01(1 - (y - this.bbox.top) / height);

      const hsv = MColor.toHSVA(this.color);
      let new_color: Color;
      if (bright_scalar === 0) {
        // Brightness is zero, color is black
        new_color = MColor.fromHSVA(hsv.h, sat_scalar, 0, this.color.a);
      } else if (sat_scalar === 0) {
        // Saturation is zero, default to red
        new_color = MColor.fromHSVA(hsv.h, 0, bright_scalar, this.color.a);
      } else {
        // General case
        const current_hsv = MColor.toHSVA(this.color);
        new_color = MColor.fromHSVA(current_hsv.h, sat_scalar, bright_scalar, this.color.a);
      }
      this.color = new_color;

      input_state.action_ret_var = this.color;
      return { wants_focus: true, action: this.action_type };
    }

    return { wants_focus: false, action: null };
  };

}
