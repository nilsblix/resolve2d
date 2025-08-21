// This is a demo-showcase of the gui. Use this as a "syntax-reference" or 
// inspiration of how to use this gui.

// All data is external to the gui, which means that the gui is immediate.
// This means that the gui automatically responds mutable data as it is 
// recreated each update. 

// Use this to import the gui.
import * as gui from "./gui/gui.ts";

// "All data is external to the gui" also means that callbacks to button/
// other interactable events are declared via this enum. An action can be
// requested each frame, and only on interaction a non-null action is returned.
enum Action {
    // The difference between Action.true and null is that Action.true is 
    // treated in the backend of stack.ts as an user-based action, which 
    // warrants a change in window-state. `null` is treated as if that specific 
    // action doesn't exist. Action.true is a generic action, which is handled 
    // internally by the gui, while `null` gets discarged and ignored by 
    // the gui.
    true,
    drag_num,
    color_picker_bg,
    open_color_picker_win,
}

// Some arbitrary gui-external data.
const app = {
    num: 0,
    bg_color_picker_win: false,
    bg_color: gui.MColor.fromHex("#4ba4c2"),
};

function update() {
    gui.updateCanvasSizing();

    const stack = new gui.Stack<gui.N<Action>>();

    const win = stack.makeWindow(gui.c, gui.input_state,
        // window is if win is moveable, header is if win is closeable and the rest are self explanatory.
        // Here we see the difference between Action.true and null.
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "Window 1", x: 500, y: 100, width: 600, height: 500 },
    );
    // Here is how to specify a widget. The parameters is different based on
    // which widget, but the principle remains the same. 
    win.makeLabel(gui.c, null, "Hello, World!");
    // One can even get a reference to the widget by assigning the return value 
    // to a variable.
    // Like this --> ```ts
    //      const label = win.makeLabel(gui.c, null, "Hello, World!");
    // ```

    // When a class is prefixed by an M, it means that the class is a namespace
    // which contains functions for the corresponding type.
    // FIXME: pls dont use 2 * padding. make M* function for this.
    // const win_width = gui.MBBox.calcWidth(win.bbox) - 2 * gui.GlobalStyle.layout_commons.padding
    const win_width = win.getInnerWidth();
    win.makeText(gui.c, null, "Hello world! This is written in `win.makeText`. I have written more text here to maybe show of some text-wrapping based on window width...", win_width);

    // Widgets gets autoformatted, with the default being in a vertical layout,
    // but can be changed to grid or two column layout etc.
    // `setMode` is used to change the layouting, but the `grid` widget is a
    // widget that just contains other widgets.
    win.makeLabel(gui.c, null, "Two column layout mode:");
    win.setMode("two columns");
    win.makeButton(gui.c, Action.true, "Left");
    win.makeButton(gui.c, Action.true, "Right");
    win.makeText(gui.c, null, "Maybe some button description: " + app.num, win_width / 2);
    win.makeDraggable(gui.c, Action.drag_num, "Draggable");
    win.setMode("normal");

    // An easy way to make a break.
    win.makeLabel(gui.c, null, "");

    win.setMode("two columns");
    win.makeText(gui.c, null, "Open color picker for background-color: ", win_width / 2);
    win.makeButton(gui.c, Action.open_color_picker_win, "Open");

    const win2 = stack.makeWindow(gui.c, gui.input_state,
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "Window 2", x: 100, y: 400, width: 200, height: 200 },
    );
    win2.makeLabel(gui.c, Action.true, "Hover over me!");

    // Here is an example of a color picker. The color chosen can be selected 
    // via `gui.input_state.action_ret_var`. See `Action.color_picker_bg` in the switch
    // statement for a better view.

    let color_picker_win_loc: gui.WidgetLoc = [];
    if (app.bg_color_picker_win) {
        const picker_win = stack.makeWindow(gui.c, gui.input_state,
            { window: Action.true, header: null, resizeable: null, close_btn: null },
            { title: "Change background-color", x: 70, y: 70, width: 420, height: 430 },
        );
        picker_win.makeColorPickerRect(Action.color_picker_bg, app.bg_color, 400, 400);
        color_picker_win_loc = picker_win.loc;
    }

    const req = stack.requestAction(gui.input_state);
    const action = req.action;

    if (color_picker_win_loc.length != 0 && !startsWith(gui.input_state.active_widget_loc, color_picker_win_loc)) {
        if (action != null) {
            app.bg_color_picker_win = false;
        }
    }

    switch (action) {
        case Action.true:
            break;
        case Action.drag_num:
            app.num += gui.input_state.mouse_delta_pos.x;
            if (app.num < -100) app.num = -100;
            if (app.num > 100) app.num = 100;
            break;
        case Action.color_picker_bg:
            if (gui.MColor.isColor(gui.input_state.action_ret_var)) {
                app.bg_color = gui.input_state.action_ret_var;
                document.body.style.backgroundColor = gui.MColor.string(app.bg_color);
            }
            break;
        case Action.open_color_picker_win:
            app.bg_color_picker_win = true;
            break;
    }

    gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
    stack.stack_render(gui.c, gui.input_state);
    gui.input_state.end();

    requestAnimationFrame(update)
}

update();

function startsWith<T>(arr: T[], prefix: T[]): boolean {
    const len = Math.min(prefix.length, arr.length);
    for (let i = 0; i < len; i++) {
        if (arr[i] != prefix[i]) {
            return false;
        }
    }

    return true;
}
