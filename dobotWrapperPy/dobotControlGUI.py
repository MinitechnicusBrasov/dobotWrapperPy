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
        self.root.geometry("600x450")
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

        # Buttons Frame
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

        # Movement Buttons Frame
        movement_frame = tk.Frame(main_frame, bg="#f0f0f0")
        movement_frame.pack(pady=10)
        create_styled_button(movement_frame, "Jump to P1", self._jump_first_cmd)
        create_styled_button(movement_frame, "Jump to P2", self._jump_second_cmd)
        create_styled_button(movement_frame, "Jump to P3", self._jump_third_cmd)

        # Schedule the initial connection attempt to run in the asyncio loop
        self.loop.call_soon_threadsafe(self.loop.create_task, self._initial_connect())

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
            raise Exception("Trying to connect")
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

    def _jump_first_cmd(self) -> None:
        """Button command for the first jump movement."""
        self._schedule_dobot_task(self.dobot.jump(19, -263, 54, 0), "Jump to P1")

    def _jump_second_cmd(self) -> None:
        """Button command for the second jump movement."""
        self._schedule_dobot_task(self.dobot.jump(262, -20, 80, 0), "Jump to P2")

    def _jump_third_cmd(self) -> None:
        """Button command for the third jump movement."""
        self._schedule_dobot_task(self.dobot.jump(4, 263, 90, 0), "Jump to P3")


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
        app = DobotGUIApp(self.root, self.loop, self.dobot_async_instance)

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
