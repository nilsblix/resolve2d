import { Stack } from "./stack.ts";

export { Stack };

const css_style = document.createElement("style");
css_style.type = "text/css";
css_style.appendChild(document.createTextNode(`
    @font-face {
    font-family: "Hack Regular";
    font-style: normal;
    font-weight: normal;
    src: local("Hack Regular"), url("/hack-webfont/Hack-Regular.woff") format("woff");
    }
    

    @font-face {
    font-family: "Hack Italic";
    font-style: normal;
    font-weight: normal;
    src: local("Hack Italic"), url("/hack-webfont/Hack-Italic.woff") format("woff");
    }
    

    @font-face {
    font-family: "Hack Bold";
    font-style: normal;
    font-weight: normal;
    src: local("Hack Bold"), url("/hack-webfont/Hack-Bold.woff") format("woff");
    }
    

    @font-face {
    font-family: "Hack Bold Italic";
    font-style: normal;
    font-weight: normal;
    src: local("Hack Bold Italic"), url("/hack-webfont/Hack-BoldItalic.woff") format("woff");
    }`
));
document.head.appendChild(css_style);

export const canvas = document.createElement("canvas");
canvas.id = "nvb-imgui-canvas";
document.body.appendChild(canvas);
canvas.style.position = "absolute";
canvas.style.top = "0px";
canvas.style.left = "0px";
export function updateCanvasSizing() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  canvas.style.width = window.innerWidth + "px";
  canvas.style.height = window.innerHeight + "px";
}

/**
* @param e num decimals to round of to
* @returns
*/
export function round(x: number, e: number): number {
  return Math.floor(x * 10 ** e) / (10 ** e);
}

function mod(a: number, b: number): number {
  if (b < 0)
    return -mod(-a, -b);
  let ret = a % b;
  if (ret < 0)
    ret += b;
  return ret;
}

export function updateDraggableValue(val: number, input_state: InputState, sensitivity: number, config?: { min?: number, max?: number }): number {
  if (config?.min != undefined && config?.max != undefined)
    console.assert(config.min < config.max);

  let v = val + sensitivity * input_state.mouse_delta_pos.x;
  if (config?.min != undefined && v < config?.min)
    v = config.min;
  if (config?.max != undefined && v > config?.max)
    v = config?.max;
  return v;
}

export type REND = CanvasRenderingContext2D;
export type N<T> = null | T;
export type WidgetLoc = number[];

export type Cursor = {
  x: number;
  y: number;
};

export type Color = {
  r: number;
  g: number;
  b: number;
  a: number;
};

export class MColor {
  static white = { r: 255, g: 255, b: 255, a: 1 };
  static black = { r: 0, g: 0, b: 0, a: 1 };
  static g_18 = { r: 18, g: 19, b: 18, a: 1 };
  static red = { r: 255, g: 0, b: 0, a: 1 };
  static default_bg: { r: 115; g: 140; b: 153; a: 1 };

  private static EPS_DECIMALS = 6;

  static isColor(x: any) {
    if (typeof x == typeof MColor.white)
      return true;
    return false;
  }

  static string(color: Color): string {
    return `rgba(${color.r}, ${color.g}, ${color.b}, ${color.a})`;
  }

  static fromString(colorString: string): Color {
    const match = colorString.match(
      /rgba?\((\d+),\s*(\d+),\s*(\d+),?\s*(\d*\.?\d+)?\)/,
    );
    if (!match) {
      throw new Error("Invalid color string format");
    }

    const [, r, g, b, a] = match;
    return {
      r: parseInt(r, 10),
      g: parseInt(g, 10),
      b: parseInt(b, 10),
      a: a !== undefined ? parseFloat(a) : 1,
    };
  }

  static fromHex(hex: string): Color {
    const hexClean = hex.replace("#", "");
    if (hexClean.length === 6) {
      // RGB format
      const r = parseInt(hexClean.slice(0, 2), 16);
      const g = parseInt(hexClean.slice(2, 4), 16);
      const b = parseInt(hexClean.slice(4, 6), 16);
      return { r, g, b, a: 1 };
    } else if (hexClean.length === 8) {
      // RGBA format
      const r = parseInt(hexClean.slice(0, 2), 16);
      const g = parseInt(hexClean.slice(2, 4), 16);
      const b = parseInt(hexClean.slice(4, 6), 16);
      const a = parseInt(hexClean.slice(6, 8), 16) / 255;
      return { r, g, b, a };
    } else {
      throw new Error("Invalid hex color format");
    }
  }

  static toHex(color: Color): string {
    const r = Math.round(color.r).toString(16).padStart(2, "0");
    const g = Math.round(color.g).toString(16).padStart(2, "0");
    const b = Math.round(color.b).toString(16).padStart(2, "0");
    const a = Math.round(color.a * 255).toString(16).padStart(2, "0");

    return color.a === 1 ? `#${r}${g}${b}` : `#${r}${g}${b}${a}`;
  }

  static fromRGB(rgb: { r: number; g: number; b: number }): Color {
    return { ...rgb, a: 1 };
  }

  static fromHSVA(h: number, s: number, v: number, a: number): Color {
    // const hp = ((h % 360) + 360) % 360;
    const hp = mod(h, 360);

    const c = v * s;
    const x = c * (1 - Math.abs((hp / 60) % 2 - 1))
    const m = v - c;

    let [r, g, b] = [0, 0, 0];

    if (0 <= hp && hp < 60)
      [r, g, b] = [c, x, 0];
    else if (60 <= hp && hp < 120)
      [r, g, b] = [x, c, 0];
    else if (120 <= hp && hp < 180)
      [r, g, b] = [0, c, x];
    else if (180 <= hp && hp < 240)
      [r, g, b] = [0, x, c];
    else if (240 <= hp && hp < 300)
      [r, g, b] = [x, 0, c];
    else if (300 <= hp && hp < 360)
      [r, g, b] = [c, 0, x];

    [r, g, b] = [255 * (r + m), 255 * (g + m), 255 * (b + m)];

    return { r: round(r, this.EPS_DECIMALS), g: round(g, this.EPS_DECIMALS), b: round(b, this.EPS_DECIMALS), a };

  }

  static toHSVA(color: Color): { h: number; s: number; v: number, a: number } {
    const r = color.r / 255;
    const g = color.g / 255;
    const b = color.b / 255;

    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const delta = max - min;

    let h = 0;
    if (delta == 0)
      h = 0;
    else if (max === r) {
      h = ((60 * ((g - b) / delta)) + 360) % 360;
    } else if (max === g) {
      h = ((60 * ((b - r) / delta)) + 120) % 360;
    } else if (max === b) {
      h = ((60 * ((r - g) / delta)) + 240) % 360;
    }

    const s = max === 0 ? 0 : delta / max;
    const v = max;

    return { h: round(h, this.EPS_DECIMALS), s: round(s, this.EPS_DECIMALS), v: round(v, this.EPS_DECIMALS), a: color.a };
  }
}

export type BBox = {
  left: number;
  top: number;
  right: number;
  bottom: number;
};

export class MBBox {
  static isInside(box: BBox, x: number, y: number): boolean {
    return x > box.left && x < box.right && y > box.top && y < box.bottom;
  }

  static calcWidth(box: BBox): number {
    return box.right - box.left;
  }

  static calcHeight(box: BBox): number {
    return box.bottom - box.top;
  }

  static set(target: BBox, src: BBox): void {
    target.left = src.left;
    target.right = src.right;
    target.top = src.top;
    target.bottom = src.bottom;
  }
}

export type Widget<ActionType> = {
  bbox: BBox;
  loc: WidgetLoc;
  action_type: ActionType;
  render: (c: REND) => void;
  requestAction: (input_state: InputState) => {
    wants_focus: boolean;
    action: N<ActionType>;
  };
};

export class GlobalStyle {
  static font = "Hack Regular";
  static widget = {
    // DEAR IMGUI
    //default_bg_color: "#294A7AFF",
    //hover_bg_color: "#4296FAFF",
    //down_bg_color: "#0F87FAFF",
    default_bg_color: "#666170FF",
    hover_bg_color: "#9992A8FF",
    down_bg_color: "#CCC2E0FF",
  }
  static label = {
    font_size: 12,
    default_font_color: MColor.white,
    inactive_font_color: MColor.fromHex("#808080FF"),
  };
  static text = {
    font_size: 12,
    text_height_mult: 1.5,
  };
  static button = {
    padding: 3,
    font_size: 12,
  };
  static layout_commons = {
    padding: 10,
    widget_gap: 5,
    bg_color: MColor.fromHex("#0F0F0FF0"),
    border: "#6E6E8080",
    border_width: 1,
  };
  static header_commons = {
    color: "#ffffff",
    //bg_color: "#294A7AFF",
    bg_color: "#4b4654",
    font_size: 12,
  };
  static window = {
    minimized_header_bg: "#9F9F9F09",
  };
  static grid = {};
  static popup = {};
}

export class InputState {
  mouse_position: Cursor;
  mouse_prev_position: Cursor;
  mouse_delta_pos: Cursor;
  mouse_down_position: Cursor;
  mouse_down: boolean;
  mouse_frame: {
    clicked: boolean;
    time_at_click: number;
    time_since_click: number;
    double_clicked: boolean;
    released: boolean;
  };
  delta_mouse_scroll = { x: 0, y: 0 };

  moving_window: boolean;
  resizing_window: boolean;

  window_offsets: Cursor[];
  window_positions: Cursor[];
  window_sizes: { width: number; height: number }[];
  window_active: boolean[];
  window_minimised: boolean[];
  window_order: number[];

  active_widget_loc: number[];

  action_ret_var: any;

  private last_click_time: number;
  private double_click_threshold: number;

  constructor(canvas: HTMLCanvasElement, x: number, y: number) {
    this.mouse_position = { x, y };
    this.mouse_prev_position = { x, y };
    this.mouse_down_position = { x, y };
    this.mouse_delta_pos = { x, y };
    this.mouse_down = false;
    this.mouse_frame = {
      clicked: false,
      time_at_click: -1,
      time_since_click: 0,
      double_clicked: false,
      released: false,
    };
    this.delta_mouse_scroll = { x: 0, y: 0 };

    this.moving_window = false;
    this.resizing_window = false;

    this.window_offsets = [];
    this.window_positions = [];
    this.window_sizes = [];
    this.window_active = [];
    this.window_minimised = [];
    this.window_order = [];
    this.active_widget_loc = [];

    this.last_click_time = -1;
    this.double_click_threshold = 300; // Double-click threshold in milliseconds

    this.action_ret_var = undefined;

    canvas.addEventListener("mousemove", (e) => {
      const rect = canvas.getBoundingClientRect();
      const x = e.clientX - rect.x;
      const y = e.clientY - rect.y;
      this.mouse_position.x = x;
      this.mouse_position.y = y;
    });

    canvas.addEventListener("mousedown", (e) => {
      const rect = canvas.getBoundingClientRect();
      const x = e.clientX - rect.x;
      const y = e.clientY - rect.y;
      this.mouse_down_position.x = x;
      this.mouse_down_position.y = y;

      const now = performance.now();

      this.mouse_frame.clicked = true;

      if (
        this.last_click_time !== -1 &&
        now - this.last_click_time < this.double_click_threshold
      ) {
        this.mouse_frame.double_clicked = true;
      } else {
        this.mouse_frame.double_clicked = false;
        if (now - this.last_click_time >= this.double_click_threshold) {
          this.last_click_time = -1;
        }
      }

      this.last_click_time = now;
      this.mouse_down = true;
    });

    canvas.addEventListener("mouseup", () => {
      this.mouse_frame.released = true;
      this.mouse_down = false;
    });

    canvas.addEventListener("wheel", (e) => {
      this.delta_mouse_scroll.x = e.deltaX;
      this.delta_mouse_scroll.y = e.deltaY;
    });
  }

  end() {
    this.mouse_delta_pos.x = this.mouse_position.x - this.mouse_prev_position.x;
    this.mouse_delta_pos.y = this.mouse_position.y - this.mouse_prev_position.y;
    this.mouse_prev_position.x = this.mouse_position.x;
    this.mouse_prev_position.y = this.mouse_position.y;

    const now = performance.now();

    if (this.mouse_frame.time_at_click !== -1) {
      this.mouse_frame.time_since_click = now - this.mouse_frame.time_at_click;
    }

    if (this.mouse_frame.clicked) {
      this.mouse_frame.time_at_click = now;
    }

    if (now - this.last_click_time >= this.double_click_threshold) {
      this.mouse_frame.time_at_click = -1;
      this.mouse_frame.time_since_click = 0;
      this.last_click_time = -1;
    }

    this.mouse_frame.clicked = false;
    this.mouse_frame.released = false;
    this.mouse_frame.double_clicked = false;

    this.delta_mouse_scroll.x = 0;
    this.delta_mouse_scroll.y = 0;
  }
}

export const c = <REND>canvas.getContext("2d");
export const input_state = new InputState(canvas, 0, 0);
