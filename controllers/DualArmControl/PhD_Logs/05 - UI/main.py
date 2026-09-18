from pathlib import Path
from datetime import date
import shutil
import re

from textual.app import App, ComposeResult
from textual.widgets import TextArea, Label, ListView, ListItem
from textual.containers import Horizontal


ROOT = Path(__file__).resolve().parent.parent
DAILY_DIR = ROOT / "01 - Daily"
TEMPLATE = DAILY_DIR / "0 - Template.md"


def ensure_today_file() -> Path:
    """Restituisce il markdown della giornata, creandolo dal template se necessario."""
    today = date.today().strftime("%Y-%m-%d")
    daily = DAILY_DIR / f"{today}.md"

    if not daily.exists():
        shutil.copy(TEMPLATE, daily)

    return daily


class JournalApp(App):

    CSS = """
    Screen {
        background: #1b1d22;
        color: white;
    }

    #main-container {
        height: 1fr;
        padding: 0;
    }

    #sidebar {
        width: 28;
        min-width: 22;

        border: round #4f8cff;
        background: #23262d;

        margin-right: 1;
    }

    .sidebar-item {
        padding: 0 0;
        height: 3;
    }

    .sidebar-item:hover {
        background: #3b4252;
    }

    ListView > .-highlight {
        background: #4f8cff;
        color: black;
        text-style: bold;
    }

    TextArea {
        border: round #4f8cff;
        background: #1f2127;

        padding: 1 2;

        width: 1fr;
        height: 100%;
    }

    Footer {
        background: #2b2f38;
    }
    """

    BINDINGS = [
        ("ctrl+s", "save", "Save"),
        ("ctrl+b", "toggle_sidebar", "Sidebar"),
        ("ctrl+q", "quit", "Quit"),
    ]

    def __init__(self, file_path: Path):
        super().__init__()

        self.file_path = file_path

        self.sections = {}
        self.section_order = []

        self.current_section = None
        self.ignore_changes = False

    def compose(self):

        with Horizontal(id="main-container"):

            yield ListView(id="sidebar")

            self.editor = TextArea.code_editor(
                language="markdown"
            )

            self.editor.show_line_numbers = False

            yield self.editor

    def on_mount(self):

        self.parse_markdown_file()

        self.populate_sidebar()

        if self.section_order:
            sidebar = self.query_one("#sidebar", ListView)
            sidebar.index = 0

    def parse_markdown_file(self):

        content = self.file_path.read_text(
            encoding="utf-8"
        )

        content = re.sub(
            r"^---\s*$",
            "",
            content,
            flags=re.MULTILINE,
        )

        raw_sections = re.split(
            r"^(##\s+.+)$",
            content,
            flags=re.MULTILINE,
        )

        header = raw_sections[0].strip()

        if header:
            self.sections["__HEADER__"] = header + "\n\n"

        for i in range(1, len(raw_sections), 2):

            title = raw_sections[i].strip()
            body = raw_sections[i + 1].strip() + "\n"

            self.sections[title] = body
            self.section_order.append(title)

    def populate_sidebar(self):

        sidebar = self.query_one("#sidebar", ListView)

        sidebar.clear()

        for title in self.section_order:

            clean = title.replace("##", "").strip()

            item = ListItem(
                Label(clean),
                classes="sidebar-item",
            )

            item.section_key = title

            sidebar.append(item)

    def on_list_view_selected(
        self,
        event: ListView.Selected,
    ):

        if (
            not event.item
            or
            not hasattr(event.item, "section_key")
        ):
            return

        if (
            self.current_section
            and
            not self.ignore_changes
        ):
            self.sections[self.current_section] = self.editor.text

        self.current_section = event.item.section_key

        self.ignore_changes = True

        self.editor.text = self.sections[self.current_section]

        self.ignore_changes = False

        self.editor.focus()

    def action_toggle_sidebar(self):

        sidebar = self.query_one("#sidebar")

        sidebar.display = not sidebar.display
    def action_save(self) -> None:
        """Salva il contenuto corrente su disco."""

        if self.current_section:
            self.sections[self.current_section] = self.editor.text

        self.save_full_file()

        self.notify(
            "✓ File salvato",
            title="Journal",
            severity="information",
            timeout=1,
        )

    def save_full_file(self) -> None:
        """Ricompone il markdown completo."""

        full_markdown = ""

        if "__HEADER__" in self.sections:
            full_markdown += self.sections["__HEADER__"]

        parts = []

        for title in self.section_order:

            content = self.sections[title].strip()

            parts.append(
                f"{title}\n\n{content}\n"
            )

        full_markdown += "\n---\n\n".join(parts)

        self.file_path.write_text(
            full_markdown,
            encoding="utf-8",
        )

    def on_unmount(self) -> None:
        """Salvataggio automatico in chiusura."""
        self.action_save()


if __name__ == "__main__":
    today_file = ensure_today_file()
    JournalApp(today_file).run()