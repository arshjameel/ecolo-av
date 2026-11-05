"""
The model test: each local model plans every request in eval_set.py, and is scored.

Run it in the llm environment (no pixi task):
pixi run -e llm tools/ros.sh python3 -m aav_language.run_eval [options]

    (no options)          the default models
    --models mlx-community/Qwen3-8B-4bit --out results.json
    --backend openai --base-url http://127.0.0.1:8080/v1 --models qwen3-4b
                          a model behind llama-server, Ollama, trtllm-serve

Each request gets a fresh pretend car: the drive server with --fake, over MCP stdio, as
the agent uses it. Only the plan is scored (nothing runs). Per model: correct plans (all,
moves, at-once, refusals), unreadable replies, seconds per model step, the most memory
MLX held, and the size on disk.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import statistics
import sys
import time

from aav_language import eval_set, react
from aav_language.agent import drive_session, server_command

MODELS = [
    'mlx-community/Llama-3.2-1B-Instruct-4bit',      # Ben's mcpsearch model
    'mlx-community/Qwen3-4B-Instruct-2507-4bit',     # the default (llm_mlx.DEFAULT_MODEL)
    'mlx-community/Qwen3.5-4B-4bit',
    'mlx-community/Qwen3-8B-4bit',
    'mlx-community/Meta-Llama-3.1-8B-Instruct-4bit',
]
# The newer models of S6 (docs/model-comparison.md): --models newer
NEWER = [
    'mlx-community/Qwen3.5-2B-4bit',
    'mlx-community/Qwen3.5-9B-4bit',
    'mlx-community/gemma-4-e2b-it-4bit',
    'mlx-community/gemma-4-e4b-it-4bit',
    'mlx-community/granite-4.1-3b-4bit',
    'mlx-community/granite-4.1-8b-4bit',
    'mlx-community/Ministral-3-3B-Instruct-2512-4bit',
    'mlx-community/Ministral-3-8B-Instruct-2512-4bit',
    'mlx-community/Llama-3.2-3B-Instruct-4bit',
    'mlx-community/gemma-4-26b-a4b-it-4bit',         # 15 GB: an Orin AGX, not an Orin Nano
    'mlx-community/Qwen3.6-35B-A3B-4bit',            # 20 GB: the same
]


def disk_gb(model_id: str) -> float:
    """Return the model's size in the Hugging Face cache, in GB (0 if not there)."""
    base = Path(os.environ.get('HF_HOME', Path.home() / '.cache' / 'huggingface')) / 'hub'
    snap = base / ('models--' + model_id.replace('/', '--')) / 'snapshots'
    total = 0
    for f in snap.rglob('*'):
        if f.is_file():
            total += f.stat().st_size      # follows the links into blobs/
    return total / 1e9


async def one_item(llm, item, json_actions: bool):
    async with drive_session(server_command(True, False, None)) as (call_tool, tools):
        status = json.loads(await call_tool('status', {}))
        agent = react.ReactAgent(llm, call_tool, react.tool_schemas(tools),
                                 status.get('limits'), json_actions=json_actions)
        plan = await agent.plan(item['say'])
    ok, why = eval_set.score(item, plan)
    return plan, ok, why


def make_llm(model_id: str, backend: str, base_url: str):
    """Load the model; return it and a function for the most memory it held (or None)."""
    if backend == 'openai':
        from aav_language.llm_openai import OpenAiChat
        return OpenAiChat(model_id, base_url), lambda: None
    from aav_language.llm_mlx import MlxChat, peak_memory_gb, reset_peak_memory
    reset_peak_memory()
    return MlxChat(model_id), lambda: round(peak_memory_gb(), 2)


async def run_model(model_id: str, json_actions: bool, verbose: bool,
                    backend: str = 'mlx', base_url: str = '') -> dict:
    t0 = time.monotonic()
    llm, peak = make_llm(model_id, backend, base_url)
    load_s = time.monotonic() - t0
    use_json = json_actions or not llm.native_tools
    rows, step_times, bad = [], [], 0
    for item in eval_set.ITEMS:
        plan, ok, why = await one_item(llm, item, use_json)
        times = [s.seconds for s in plan.steps]
        step_times += times
        bad += sum(1 for s in plan.steps if s.call is None and s.observation.startswith('error'))
        rows.append({'id': item['id'], 'say': item['say'], 'kind': eval_set.kind(item),
                     'ok': ok, 'why': why, 'steps': len(plan.steps),
                     'seconds': round(sum(times), 2),
                     'queued': [c.text() for c in plan.queued],
                     'unsaid': plan.unsaid(),
                     'ran_now': [c.text() for c in plan.ran_now],
                     'answer': plan.answer[:300],
                     'raw': [s.raw[:400] for s in plan.steps]})
        mark = 'ok ' if ok else 'BAD'
        print(f'  {mark} {item["id"]:10s} {sum(times):5.1f} s  {why}', flush=True)
        if verbose and not ok:
            for s in plan.steps:
                print('       raw:', s.raw[:300].replace('\n', ' | '))
    by_kind = {}
    for k in ('move', 'now', 'refuse'):
        sel = [r for r in rows if r['kind'] == k]
        by_kind[k] = f'{sum(r["ok"] for r in sel)}/{len(sel)}'
    wrong = [r for r in rows if not r['ok'] and r['queued']]
    flagged = f'{sum(bool(r["unsaid"]) for r in wrong)}/{len(wrong)}'
    return {'model': model_id, 'format': 'json actions' if use_json else 'native tools',
            'wrong_plans_flagged': flagged,
            'correct': sum(r['ok'] for r in rows), 'total': len(rows), 'by_kind': by_kind,
            'unreadable_replies': bad, 'load_s': round(load_s, 1),
            'step_s_median': round(statistics.median(step_times), 2) if step_times else None,
            'step_s_max': round(max(step_times), 2) if step_times else None,
            'request_s_mean': round(sum(r['seconds'] for r in rows) / len(rows), 2),
            'peak_memory_gb': peak(), 'disk_gb': round(disk_gb(model_id), 2),
            'backend': backend,
            'rows': rows}


def table(results) -> str:
    lines = ['| Model | Correct | Moves | At once | Refusals | Wrong plans flagged '
             '| s/step (median) | s/request | Memory GB | Disk GB |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        lines.append(
            f'| {r["model"].split("/")[-1]} | **{r["correct"]}/{r["total"]}** | '
            f'{r["by_kind"]["move"]} | {r["by_kind"]["now"]} | {r["by_kind"]["refuse"]} | '
            f'{r["wrong_plans_flagged"]} | {r["step_s_median"]} | {r["request_s_mean"]} | '
            f'{r["peak_memory_gb"]} | {r["disk_gb"]} |')
    return '\n'.join(lines)


async def main_async(args):
    results = []
    for model_id in args.models:
        print(f'== {model_id}', flush=True)
        try:
            results.append(await run_model(model_id, args.json_actions, args.verbose,
                                           args.backend, args.base_url))
        except Exception as e:  # noqa: B902 (a model that won't load: say why, go on)
            print(f'   could not run {model_id}: {type(e).__name__}: {str(e)[:300]}',
                  flush=True)
            continue
        print(table(results[-1:]), flush=True)
        if args.out:
            Path(args.out).write_text(json.dumps(results, indent=1))
    print('\n' + table(results))


def main(argv=None):
    p = argparse.ArgumentParser(description='Score local models on the plain-English test.')
    p.add_argument('--models', nargs='+', default=MODELS,
                   help="model ids; 'newer' = the S6 list (NEWER)")
    p.add_argument('--backend', choices=['mlx', 'openai'], default='mlx',
                   help='mlx: in this process (a Mac); openai: an OpenAI-compatible server')
    p.add_argument('--base-url', default='http://127.0.0.1:8080/v1',
                   help='the server, with --backend openai (llm_openai.py)')
    p.add_argument('--out', help='also write every plan and score to this JSON file')
    p.add_argument('--json-actions', action='store_true',
                   help="JSON actions for every model, not the model's own tool format")
    p.add_argument('--only', nargs='+', help='only these item ids (eval_set.py)')
    p.add_argument('-v', '--verbose', action='store_true')
    args = p.parse_args(argv)
    args.models = [m for name in args.models for m in (NEWER if name == 'newer' else [name])]
    if args.only:
        eval_set.ITEMS[:] = [i for i in eval_set.ITEMS if i['id'] in args.only]
    asyncio.run(main_async(args))
    return 0


if __name__ == '__main__':
    sys.exit(main())
