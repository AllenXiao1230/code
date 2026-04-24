# main.py
import importlib.util
import sys


def _missing_runtime_deps() -> list[tuple[str, str]]:
    required = [
        ("PyQt5", "PyQt5"),
        ("serial", "pyserial"),
    ]
    missing = []
    for module_name, pip_name in required:
        if importlib.util.find_spec(module_name) is None:
            missing.append((module_name, pip_name))
    return missing


def main() -> int:
    missing = _missing_runtime_deps()
    if missing:
        names = ", ".join(module_name for module_name, _ in missing)
        install_cmd = f"{sys.executable} -m pip install -r requirements.txt"
        print(f"Missing dependencies: {names}. Run: {install_cmd}")
        return 1

    from PyQt5.QtWidgets import QApplication
    from gui_main import MainWindow

    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
