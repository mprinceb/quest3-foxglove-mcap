# Repository Guidelines

## SLAM Work Notice
For SLAM implementation tasks (Quest 3 capture -> SLAM), follow `SLAM_AGENT_GUIDE.md` as the primary execution playbook.

## VIO Work Notice
For VIO implementation tasks (Quest 3 capture -> VIO), follow `VIO_AGENT_GUIDE.md` as the primary execution playbook.

## Project Structure & Module Organization
This repository is a small Python CLI for converting Quest 3 capture exports into Foxglove-compatible MCAP files.

- `main.py`: primary converter entrypoint and CLI (`--dir`, `--out`, `--res`, `--no-raw`).
- `script1.py`: legacy/prototype conversion script; keep changes focused on `main.py` unless maintaining legacy behavior.
- `data/`: local input capture data (ignored by git).
- `out/`: generated `.mcap` outputs (ignored by git).
- `axes_yz_swap.md`: coordinate-frame rationale used by the transform logic.
- `pyproject.toml` and `uv.lock`: dependency and environment definition.

## Build, Test, and Development Commands
- `uv sync`: install/update dependencies from `pyproject.toml` and `uv.lock`.
- `uv run python main.py --help`: view CLI options.
- `uv run python main.py --dir data/<capture_dir> --out my_run --res 720x720`: run a conversion and write `out/my_run.mcap`.
- `uv run python main.py --dir data/<capture_dir> --out my_run --no-raw`: generate compressed-image topics only.
- `uv run python -m py_compile main.py script1.py`: quick syntax validation.

## Coding Style & Naming Conventions
- Follow PEP 8 with 4-space indentation and clear, short helper functions.
- Use `snake_case` for functions/variables and descriptive argument names.
- Keep topic names and frame IDs explicit (for example `"/camera/left/image"` and `"camera_left"`).
- Prefer small, testable functions for parsing, transforms, and message writing.

## Testing Guidelines
No formal test suite exists yet. For each change:
- run `uv run python -m py_compile main.py script1.py`;
- run one end-to-end conversion on a representative dataset in `data/`;
- verify a new file appears under `out/` and that expected topics are present in Foxglove.

When adding tests later, place them under `tests/` and name files `test_<module>.py`.

## Commit & Pull Request Guidelines
- Use Conventional Commit style observed in history: `feat: ...`, `chore: ...`.
- Keep commits focused and functional (one logical change per commit).
- PRs should include: purpose, key behavior changes, commands run for validation, and sample output path(s) (for example `out/<name>.mcap`).
- Link related issues and include screenshots only when visualization behavior changes.
