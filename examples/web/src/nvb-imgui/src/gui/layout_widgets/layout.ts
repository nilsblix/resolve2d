import { REND, N, WidgetLoc, Cursor, Color, BBox, MBBox, Widget, GlobalStyle, InputState } from "../gui.ts";
import { Label } from "../basic_widgets/label.ts";
import { Button } from "../basic_widgets/button.ts";
import { Draggable } from "../basic_widgets/draggable.ts";
import { Text } from "../basic_widgets/text.ts";
import { ColorPickerRect } from "../basic_widgets/color_picker.ts";

import { WindowHeader } from "./window.ts";

export abstract class Layout<ActionType> implements Widget<ActionType> {
  bbox: BBox;
  action_type: ActionType;
  loc: WidgetLoc;
  widgets: Widget<ActionType>[];
  cursor: Cursor;

  constructor(action_type: ActionType, loc: WidgetLoc, x: number, y: number, width: number, height: number) {
    this.cursor = { x: x + GlobalStyle.layout_commons.padding, y: y };
    this.bbox = { left: x, top: y, right: x + width, bottom: y + height };
    this.action_type = action_type;
    this.loc = loc;
    this.widgets = [];
  }

  abstract updateBBox(widget: Widget<ActionType>): void;

  getInnerWidth(): number {
    return MBBox.calcWidth(this.bbox) - 2 * GlobalStyle.layout_commons.padding;
  }

  pushWidget(widget: Widget<ActionType>) {
    this.widgets.push(widget);
  }

  render(c: REND): void {
    for (let i = this.widgets.length - 1; i >= 0; i--) {
      const widget = this.widgets[i];
      widget.render(c);
    }
  }

  requestAction(input_state: InputState): { iters: N<number>, wants_focus: boolean, action: N<ActionType> } {
    const is_candidate_for_active = (widget: Widget<ActionType>) => {
      if (input_state.active_widget_loc.length === 0) return true;
      const activePath = input_state.active_widget_loc.slice(0, widget.loc.length);
      return activePath.length === widget.loc.length && activePath.every((v, i) => v === widget.loc[i]);
    };

    // FIXME: this might have to be activated when working with popups again
    // if (!is_candidate_for_active(this)) {
    //   return { wants_focus: false, action: null };
    // }

    const [x, y] = [input_state.mouse_position.x, input_state.mouse_position.y];
    const inside = MBBox.isInside(this.bbox, x, y);

    if (!inside && input_state.active_widget_loc.length == 0)
      return { iters: null, wants_focus: false, action: null };

    if (!input_state.moving_window) {
      for (let i = 0; i < this.widgets.length; i++) {
        const widget = this.widgets[i];
        if (!is_candidate_for_active(widget)) {
          continue;
        }
        const ret = widget.requestAction(input_state);
        if (ret.wants_focus || ret.action != null || (!(widget instanceof Layout) && MBBox.isInside(widget.bbox, x, y) && widget.action_type != null && !(widget instanceof WindowHeader)))
          return { iters: i, wants_focus: ret.wants_focus, action: ret.action };
      }
    }

    if (input_state.active_widget_loc.length > 1 && JSON.stringify(input_state.active_widget_loc) != JSON.stringify(this.loc)) {
      input_state.active_widget_loc = [];
      return { iters: null, wants_focus: false, action: null };
    }

    if (this.action_type == null)
      return { iters: null, wants_focus: false, action: null };

    if (inside && input_state.mouse_frame.clicked) {
      input_state.moving_window = true;
      input_state.active_widget_loc = this.loc;
    }

    if (input_state.moving_window && input_state.mouse_frame.released) {
      input_state.moving_window = false;
      input_state.active_widget_loc = [];
    }

    return { iters: null, wants_focus: false, action: null };

  };

  makeLabel(c: REND, action_type: ActionType, text: string): Label<ActionType> {
    const label = new Label<ActionType>(c, action_type, this.loc.concat([this.widgets.length]), this.cursor, text);
    this.pushWidget(label);
    return label;
  }

  makeButton(c: REND, action_type: ActionType, text: string, config?: {width?: number, height?: number}): Button<ActionType> {
    const button = new Button<ActionType>(c, action_type, this.loc.concat([this.widgets.length]), this.cursor, text, config?.width, config?.height);
    this.pushWidget(button);
    return button;
  }

  makeDraggable(c: REND, action_type: ActionType, text: string, config?: {width?: number}): Draggable<ActionType> {
    const draggable = new Draggable<ActionType>(c, action_type, this.loc.concat([this.widgets.length]), this.cursor, text, config?.width);
    this.pushWidget(draggable);
    return draggable;
  }

  makeText(c: REND, action_type: ActionType, text: string, max_width?: number): Text<ActionType> {
    const wtext = new Text<ActionType>(c, action_type, this.loc.concat([this.widgets.length]), this.cursor, text, max_width ?? (MBBox.calcWidth(this.bbox) - 2 * GlobalStyle.layout_commons.padding));
    this.pushWidget(wtext);
    return wtext;
  }

  makeColorPickerRect(action_type: ActionType, color: Color, width: number, height: number): ColorPickerRect<ActionType> {
    const picker = new ColorPickerRect<ActionType>(action_type, this.loc.concat([this.widgets.length]), this.cursor, color, width, height);
    this.pushWidget(picker);
    return picker;
  }

  resetCursor() {
    this.cursor.x = this.bbox.left + 2 * GlobalStyle.layout_commons.widget_gap;
    this.cursor.y = this.widgets[this.widgets.length - 1].bbox.bottom + GlobalStyle.layout_commons.widget_gap;
  }

}
