import { REND, WidgetLoc, MBBox, MColor, Widget, GlobalStyle, InputState, N } from "../gui.ts";
import { Layout } from "./layout.ts";

export class Grid<ActionType> extends Layout<ActionType> implements Widget<ActionType> {
  columns: number;

  constructor(action_type: ActionType, loc: WidgetLoc, x: number, y: number, width: number, columns: number) {
    super(action_type, loc, x, y, width, 0);

    this.columns = columns;
  }

  updateBBox(widget: Widget<ActionType>): void {
    const p = GlobalStyle.layout_commons.padding;

    if (widget.bbox.top < this.bbox.top + p) {
      this.bbox.top = widget.bbox.top - p;
    } if (widget.bbox.bottom > this.bbox.bottom - p) {
      this.bbox.bottom = widget.bbox.bottom + p;
    }
  }

  pushWidget(widget: Widget<ActionType>): void {
    super.pushWidget(widget);
    this.updateBBox(widget);

    const col = this.widgets.length % this.columns;
    if (col == 0) { // ==> switch row
      let bot = Number.NEGATIVE_INFINITY;
      for (let i = this.widgets.length - this.columns; i < this.widgets.length; i++) {
        if (this.widgets[i].bbox.bottom > bot)
          bot = this.widgets[i].bbox.bottom;
      }
      this.cursor.x = this.bbox.left + GlobalStyle.layout_commons.padding;
      this.cursor.y = bot + GlobalStyle.layout_commons.widget_gap;
      return;
    }

    this.cursor.x += MBBox.calcWidth(this.bbox) / this.columns;

  }

  render(c: REND): void {
    c.fillStyle = MColor.string(GlobalStyle.layout_commons.bg_color);
    c.fillRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));
    c.strokeStyle = GlobalStyle.layout_commons.border;
    c.lineWidth = 1;
    c.strokeRect(this.bbox.left, this.bbox.top, MBBox.calcWidth(this.bbox), MBBox.calcHeight(this.bbox));

    super.render(c);
  }

  requestAction(input_state: InputState): { iters: N<number>, wants_focus: boolean; action: N<ActionType>; } {
    const is_candidate_for_active = (widget: Widget<ActionType>) => {
      if (input_state.active_widget_loc.length === 0) return true;
      const activePath = input_state.active_widget_loc.slice(0, widget.loc.length);
      return activePath.length === widget.loc.length && activePath.every((v, i) => v === widget.loc[i]);
    };

    if (!input_state.moving_window) {
      for (let i = 0; i < this.widgets.length; i++) {
        const widget = this.widgets[i];
        if (!is_candidate_for_active(widget)) {
          continue;
        }
        const ret = widget.requestAction(input_state);
        if (ret.wants_focus || ret.action != null)
          return { iters: i, ...ret };
      }
    }

    return { iters: null, wants_focus: false, action: null };

  }

}
