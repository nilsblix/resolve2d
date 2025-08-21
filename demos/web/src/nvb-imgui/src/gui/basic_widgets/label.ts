import { REND, N, WidgetLoc, Cursor, BBox, MBBox, MColor, Widget, GlobalStyle, InputState } from "../gui.ts";

const enum LabelState {
  default,
  hovered,
}

export class Label<ActionType> implements Widget<ActionType> {
  bbox: BBox;
  action_type: ActionType;
  loc: WidgetLoc;
  text: string;
  text_size: number;
  state: LabelState;

  constructor(c: REND, action_type: ActionType, loc: WidgetLoc, cursor: Cursor, text: string) {
    c.font = GlobalStyle.label.font_size + "px " + GlobalStyle.font;
    c.textBaseline = "middle";
    const m = c.measureText(text);
    const width = m.width;
    const height = m.fontBoundingBoxAscent + m.fontBoundingBoxDescent;
    this.bbox = { left: cursor.x, top: cursor.y, right: cursor.x + width, bottom: cursor.y + height };
    this.text = text;
    this.action_type = action_type;
    this.text_size = GlobalStyle.label.font_size;
    this.state = LabelState.default;
    this.loc = loc;
  }

  render(c: REND): void {
    let color = MColor.string(MColor.white);
    if (this.state == LabelState.hovered)
      color = GlobalStyle.widget.hover_bg_color;

    c.font = this.text_size + "px " + GlobalStyle.font;
    c.fillStyle = color;
    c.textBaseline = "middle";
    c.textAlign = "center";

    const x = (this.bbox.left + this.bbox.right) / 2;
    const y = (this.bbox.top + this.bbox.bottom) / 2;
    c.fillText(this.text, x, y);

    //TEMP DEBUG
    //c.strokeStyle = MColor.string(MColor.white);
    //c.lineWidth = 1;
    //c.strokeRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));

  }

  requestAction(input_state: InputState): { wants_focus: boolean, action: N<ActionType> } {
    if (this.action_type == null)
      return { wants_focus: false, action: null };
    const [x, y] = [input_state.mouse_position.x, input_state.mouse_position.y];

    if (MBBox.isInside(this.bbox, x, y) && this.action_type != null) {
      this.state = LabelState.hovered;
    }

    if (this.state == LabelState.hovered) {
      return { wants_focus: input_state.mouse_frame.clicked, action: this.action_type };
    }

    return { wants_focus: false, action: null };

  };

}
