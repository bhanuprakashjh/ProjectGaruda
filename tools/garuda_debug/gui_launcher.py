"""PyInstaller entry point for the packaged garuda-gui executable.

One exe, two roles:
  garuda-gui.exe            -> the GUI (auto-starts the broker as a child)
  garuda-gui.exe --broker   -> the broker (spawned by the GUI when frozen;
                               `python -m garuda_gsp.broker` doesn't exist
                               inside a frozen bundle)
"""
import sys


def main():
    if "--broker" in sys.argv:
        sys.argv.remove("--broker")
        from garuda_gsp.broker import main as broker_main
        broker_main()
    else:
        from garuda_gui.app import main as gui_main
        gui_main()


if __name__ == "__main__":
    main()
