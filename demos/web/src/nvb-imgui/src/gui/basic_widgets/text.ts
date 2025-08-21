import { REND, N, WidgetLoc, Cursor, BBox, MBBox, MColor, Widget, GlobalStyle, InputState } from "../gui.ts";

const enum TextState {
  default,
  hovered,
}

export class Text<ActionType> implements Widget<ActionType> {
  bbox: BBox;
  action_type: ActionType;
  loc: WidgetLoc;
  text: string;
  text_size: number;
  max_width: number; // New property for text wrapping
  state: TextState;
  wrapped_lines: string[]; // Stores wrapped lines of text

  constructor(c: REND, action_type: ActionType, loc: WidgetLoc, cursor: Cursor, text: string, max_width: number) {
    c.font = GlobalStyle.text.font_size + "px " + GlobalStyle.font;
    c.textBaseline = "middle";

    this.max_width = max_width;
    this.text = text;
    this.action_type = action_type;
    this.text_size = GlobalStyle.text.font_size;
    this.state = TextState.default;
    this.loc = loc;

    // Calculate wrapped lines
    this.wrapped_lines = this.wrapText(c, text, max_width);

    let width = 0;
    for (let i = 0; i < this.wrapped_lines.length; i++) {
      const w = c.measureText(this.wrapped_lines[i]).width
      if (width < w)
        width = w;

    }

    // Calculate bounding box based on wrapped lines
    const line_height = this.text_size * GlobalStyle.text.text_height_mult;
    const height = this.wrapped_lines.length * line_height;
    this.bbox = { left: cursor.x, top: cursor.y, right: cursor.x + width, bottom: cursor.y + height };
  }

  // Method to wrap text into lines
  wrapText(c: REND, text: string, max_width: number): string[] {
    const words = text.split(" ");
    const lines: string[] = [];
    let current_line = "";

    for (let word of words) {
      const test_line = current_line + (current_line ? " " : "") + word;
      const metrics = c.measureText(test_line);
      if (metrics.width > max_width) {
        if (current_line) lines.push(current_line);
        current_line = word;
      } else {
        current_line = test_line;
      }
    }

    if (current_line) lines.push(current_line);
    return lines;
  }

  render(c: REND): void {
    let color = MColor.string(MColor.white);
    if (this.state == TextState.hovered) color = GlobalStyle.widget.hover_bg_color;

    c.font = this.text_size + "px " + GlobalStyle.font;
    c.fillStyle = color;
    c.textBaseline = "middle";
    c.textAlign = "left"; // Align text to the left

    const line_height = this.text_size * 1.2; // Adjust line height as needed
    let y = this.bbox.top + line_height / 2;

    for (let line of this.wrapped_lines) {
      const x = this.bbox.left; // Start text at the left edge of the bounding box
      c.fillText(line, x, y);
      y += line_height;
    }

    //TEMP DEBUG
    //c.strokeStyle = MColor.string(MColor.white);
    //c.lineWidth = 1;
    //c.strokeRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));
  }

  requestAction(input_state: InputState): { wants_focus: boolean, action: N<ActionType> } {
    if (this.action_type == null) return { wants_focus: false, action: null };

    const [x, y] = [input_state.mouse_position.x, input_state.mouse_position.y];

    if (MBBox.isInside(this.bbox, x, y) && this.action_type != null) {
      this.state = TextState.hovered;
    }

    if (this.state == TextState.hovered) {
      return { wants_focus: input_state.mouse_frame.clicked, action: this.action_type };
    }

    return { wants_focus: false, action: null };
  }
}
