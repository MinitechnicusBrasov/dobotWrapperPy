from .dobotasync import DobotAsync
import asyncio
import tkinter as tk
from typing import Callable, Awaitable, Any, Dict
import threading


class DobotGUIApp:
    """
    A Tkinter GUI application integrated with asyncio for controlling a Dobot.
    """

    def __init__(
        self, root: tk.Tk, loop: asyncio.AbstractEventLoop, dobot_instance: DobotAsync
    ):
        self.root = root
        self.loop = loop
        self.dobot = dobot_instance

        self.root.title("Dobot Control Panel")
        self.root.geometry("800x800")  # Increased height to accommodate new controls
        self.root.resizable(False, False)  # Fixed size for simplicity, can be changed

        # Apply Inter font globally (or as much as Tkinter allows)
        self.root.option_add("*Font", "Inter 12")

        # Main frame for layout
        main_frame = tk.Frame(self.root, padx=20, pady=20, bg="#f0f0f0")
        main_frame.pack(expand=True, fill="both")

        # Status Label
        self.status_label = tk.Label(
            main_frame,
            text="Initializing Dobot...",
            font=("Inter", 16, "bold"),
            fg="#333",
            bg="#f0f0f0",
            wraplength=550,
        )
        self.status_label.pack(pady=(0, 20))

        # Connection Status Label
        self.connection_status_label = tk.Label(
            main_frame,
            text="Connection: Disconnected",
            font=("Inter", 12),
            fg="red",
            bg="#f0f0f0",
        )
        self.connection_status_label.pack(pady=(0, 10))

        # Buttons Frame (for general Dobot commands)
        buttons_frame = tk.Frame(main_frame, bg="#f0f0f0")
        buttons_frame.pack(pady=10)

        # Helper for creating styled buttons
        def create_styled_button(
            parent: tk.Widget, text: str, command: Callable[[], None]
        ) -> tk.Button:
            btn = tk.Button(
                parent,
                text=text,
                command=command,
                font=("Inter", 11, "bold"),
                bg="#007bff",
                fg="white",
                activebackground="#0056b3",
                relief="raised",
                bd=3,
                padx=15,
                pady=8,
                borderwidth=2,
                highlightbackground="#0056b3",
                cursor="hand2",
            )
            btn.pack(side="left", padx=10, pady=5, ipadx=5, ipady=2)

            # Explicitly typed event handlers for hover effects
            def on_enter(event: Any) -> None:
                btn.config(bg="#0056b3")

            def on_leave(event: Any) -> None:
                btn.config(bg="#007bff")

            btn.bind("<Enter>", on_enter)
            btn.bind("<Leave>", on_leave)
            return btn

        # Dobot Control Buttons
        create_styled_button(buttons_frame, "Connect Dobot", self._connect_dobot_cmd)
        create_styled_button(buttons_frame, "Clear Alarms", self._clear_alarms_cmd)
        create_styled_button(buttons_frame, "Get Pose", self._get_pose_cmd)

        # Movement Buttons Frame (for preset jumps)
        # movement_frame = tk.Frame(main_frame, bg="#f0f0f0")
        # movement_frame.pack(pady=10)

        # Joystick Frame for incremental movements
        joystick_frame = tk.LabelFrame(
            main_frame,
            text="Incremental Movement (Joystick)",
            padx=10,
            pady=10,
            bg="#f0f0f0",
            font=("Inter", 12, "bold"),
        )
        joystick_frame.pack(pady=20, fill="x")

        # Use a grid layout for joystick-like appearance
        joystick_grid_frame = tk.Frame(joystick_frame, bg="#f0f0f0")
        joystick_grid_frame.pack(expand=True)  # Center the grid within the labelframe

        # Define button size for consistent joystick look
        button_width = 8
        button_height = 2

        # Helper for creating styled buttons for grid layout, with fixed size
        def create_styled_button_grid(
            parent: tk.Widget,
            text: str,
            command: Callable[[], None],
            width: int,
            height: int,
        ) -> tk.Button:
            btn = tk.Button(
                parent,
                text=text,
                command=command,
                font=("Inter", 11, "bold"),
                bg="#007bff",
                fg="white",
                activebackground="#0056b3",
                relief="raised",
                bd=3,
                width=width,
                height=height,  # Fixed width and height
                borderwidth=2,
                highlightbackground="#0056b3",
                cursor="hand2",
            )

            # Basic hover effect
            def on_enter(event: Any) -> None:
                btn.config(bg="#0056b3")

            def on_leave(event: Any) -> None:
                btn.config(bg="#007bff")

            btn.bind("<Enter>", on_enter)
            btn.bind("<Leave>", on_leave)
            return btn

        # Row 0: Z-axis control
        tk.Frame(
            joystick_grid_frame,
            width=button_width * 10,
            height=button_height * 10,
            bg="#f0f0f0",
        ).grid(
            row=0, column=0, columnspan=3
        )  # Spacer
        create_styled_button_grid(
            joystick_grid_frame, "+Y", self._positive_y, button_width, button_height
        ).grid(row=0, column=1, padx=5, pady=5)

        # Row 1: X-axis and Y-axis controls
        create_styled_button_grid(
            joystick_grid_frame, "-X", self._negative_x, button_width, button_height
        ).grid(row=1, column=0, padx=5, pady=5)
        create_styled_button_grid(
            joystick_grid_frame, "+X", self._positive_x, button_width, button_height
        ).grid(row=1, column=2, padx=5, pady=5)
        # Add a blank space in the center of the X-Y-Z cross
        tk.Frame(
            joystick_grid_frame,
            width=button_width * 10,
            height=button_height * 10,
            bg="#f0f0f0",
        ).grid(row=1, column=1, padx=5, pady=5)

        # Row 2: Z-axis and R-axis controls
        create_styled_button_grid(
            joystick_grid_frame, "-Y", self._negative_z, button_width, button_height
        ).grid(row=2, column=1, padx=5, pady=5)

        # Separate frame for Y and R controls to allow more flexible placement
        y_r_frame = tk.Frame(joystick_grid_frame, bg="#f0f0f0")
        y_r_frame.grid(
            row=1, column=3, rowspan=2, padx=15, pady=5, sticky="ns"
        )  # Placed to the right of the main cross

        create_styled_button_grid(
            y_r_frame, "+Z", self._positive_z, button_width, button_height
        ).pack(pady=5)
        create_styled_button_grid(
            y_r_frame, "-Z", self._negative_z, button_width, button_height
        ).pack(pady=5)
        create_styled_button_grid(
            y_r_frame, "+R", self._positive_r, button_width, button_height
        ).pack(pady=5)
        create_styled_button_grid(
            y_r_frame, "-R", self._negative_r, button_width, button_height
        ).pack(pady=5)

    async def _initial_connect(self) -> None:
        """Attempts to connect to Dobot on app startup."""
        self.status_label.config(text="Attempting to connect to Dobot...")
        success: bool = await self._connect_dobot_task()
        if not success:
            self.status_label.config(
                text="Auto-connection failed. Click 'Connect Dobot'.", fg="orange"
            )
            self.connection_status_label.config(
                text="Connection: Disconnected", fg="red"
            )
        else:
            self.status_label.config(
                text="Dobot connected. Ready for commands.", fg="green"
            )
            self.connection_status_label.config(
                text="Connection: Connected", fg="green"
            )

    def _schedule_dobot_task(self, task_coro: Awaitable[Any], description: str) -> None:
        """
        Helper to schedule an asynchronous Dobot task and update the GUI status.
        This method is called from the Tkinter thread, so it uses call_soon_threadsafe.
        """

        async def wrapper() -> None:
            self.status_label.config(text=f"{description} started...", fg="blue")
            print(f"Task: {description} started.")
            try:
                await task_coro
                self.status_label.config(text=f"{description} completed!", fg="green")
                print(f"Task: {description} completed.")
            except Exception as e:
                self.status_label.config(text=f"{description} failed: {e}", fg="red")
                print(f"Error during {description}: {e}")
            finally:
                pass  # Color will be reset by next action or stay red if error

        # Schedule the async wrapper to run in the asyncio loop
        self.loop.call_soon_threadsafe(self.loop.create_task, wrapper())

    async def _connect_dobot_task(self) -> bool:
        """Asynchronous task to connect to the Dobot."""
        try:
            await self.dobot.get_device_id()
            self.connection_status_label.config(
                text="Connection: Connected", fg="green"
            )
            return True
        except Exception as e:
            self.connection_status_label.config(text="Connection: Failed", fg="red")
            raise e  # Re-raise to be caught by _schedule_dobot_task wrapper

    def _connect_dobot_cmd(self) -> None:
        """Button command for connecting to Dobot."""
        self._schedule_dobot_task(self._connect_dobot_task(), "Connecting Dobot")

    def _clear_alarms_cmd(self) -> None:
        """Button command for clearing Dobot alarms."""
        self._schedule_dobot_task(self.dobot.clear_alarms(), "Clearing Alarms")

    def _get_pose_cmd(self) -> None:
        """Button command for getting Dobot's current pose."""

        async def get_pose_coro() -> None:
            pose_data = await self.dobot.pose()
            self.status_label.config(
                text=f"Current Pose: X:{pose_data[0]:.1f}, Y:{pose_data[1]:.1f}, Z:{pose_data[2]:.1f}, R:{pose_data[3]:.1f}",
                fg="purple",
            )

        self._schedule_dobot_task(get_pose_coro(), "Getting Pose")

    def _positive_x(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_positive_x(), "+X")

    def _negative_x(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_negative_x(), "-X")

    def _positive_y(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_positive_y(), "+Y")

    def _negative_y(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_negative_y(), "-Y")

    def _positive_z(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_positive_z(), "+Z")

    def _negative_z(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_negative_z(), "-Z")

    def _positive_r(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_positive_r(), "+R")

    def _negative_r(self) -> None:
        self._schedule_dobot_task(self.dobot.move_joystick_negative_r(), "-R")


class DobotGUIController:
    """
    Manages the DobotAsync instance and provides a method to initialize the GUI.
    This class acts as the main entry point for the programmer.
    """

    def __init__(self, dobot: DobotAsync, use_async: bool = True):
        self.dobot_async_instance: DobotAsync = dobot
        self.loop: asyncio.AbstractEventLoop | None = None
        self.async_thread: threading.Thread | None = None
        self.root: tk.Tk | None = None

    def _run_async_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Runs the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def initialize_gui(self) -> None:
        """
        Initializes and runs the Tkinter GUI for Dobot control.
        This method ensures the asyncio loop runs in a separate thread.
        """
        if self.root and self.root.winfo_exists():
            print("GUI is already running.")
            return

        self.root = tk.Tk()
        # Set up a protocol for when the window is closed
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # Get or create an asyncio event loop for the async thread
        self.loop = asyncio.new_event_loop()

        # Start the asyncio loop in a separate thread
        self.async_thread = threading.Thread(
            target=self._run_async_loop, args=(self.loop,), daemon=True
        )
        self.async_thread.start()

        # Create and run the GUI application in the main thread
        DobotGUIApp(self.root, self.loop, self.dobot_async_instance)
        #
        # Start the Tkinter main loop
        self.root.mainloop()

        # After Tkinter mainloop exits, ensure asyncio loop is stopped
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.async_thread and self.async_thread.is_alive():
            self.async_thread.join(timeout=5)  # Wait for the thread to finish

    def _on_closing(self) -> None:
        """Handles the window closing event."""
        print("Closing Tkinter window. Stopping asyncio loop...")
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.root is not None:
            self.root.destroy()

    # Expose DobotAsync methods for direct programmer calls
    # These methods will be scheduled on the asyncio loop.
