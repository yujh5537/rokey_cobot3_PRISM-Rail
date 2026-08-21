"""
topology.py — 레일 네트워크를 그래프로 읽어들이고 경로를 찾는 모듈.

초보자용 설명
------------
'노드(Node)'는 캡슐이 설 수 있는 지점, '블록(Block)'은 노드와 노드를 잇는 구간입니다.
레일 관제에서 가장 중요한 규칙은 딱 하나입니다.

    "한 블록에는 캡슐이 동시에 1대만 들어갈 수 있다."

이걸 폐색(Block) 제어라고 부르고, 충돌과 교착을 막는 최후의 안전망입니다.
경로 탐색은 노드 10개짜리 아주 작은 그래프이므로 BFS(너비 우선 탐색)로 충분합니다.
(A* / WHCA*는 노드가 수십 개로 늘어난 뒤에 도입해도 늦지 않습니다.)
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class Block:
    """레일 구간 하나. 상호배제(mutex) 자원으로 취급합니다."""
    bid: str
    frm: str
    to: str
    traverse_s: float
    bottleneck: bool = False
    allow_evacuate: bool = True     # False = 단선·중간 이탈 불가(수직 쉬프트)
    forbid: list[str] = field(default_factory=list)
    via: list[str] = field(default_factory=list)

    def other_end(self, node: str) -> str:
        """이 블록의 반대쪽 끝 노드를 돌려줍니다(양방향 레일 가정)."""
        return self.to if node == self.frm else self.frm


class Topology:
    def __init__(self, path: str | Path):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.meta: dict = data.get("meta", {})
        self.nodes: dict[str, dict] = data["nodes"]
        self.sidings: dict[str, str] = data.get("sidings", {})

        self.blocks: dict[str, Block] = {}
        for bid, b in data["blocks"].items():
            self.blocks[bid] = Block(
                bid=bid,
                frm=b["from"],
                to=b["to"],
                traverse_s=float(b["traverse_s"]),
                bottleneck=bool(b.get("bottleneck", False)),
                allow_evacuate=bool(b.get("allow_evacuate", True)),
                forbid=list(b.get("forbid", [])),
                via=list(b.get("via", [])),
            )

        # 인접 리스트: node -> [(이웃노드, 블록id), ...]
        self.adj: dict[str, list[tuple[str, str]]] = {n: [] for n in self.nodes}
        for blk in self.blocks.values():
            self.adj[blk.frm].append((blk.to, blk.bid))
            self.adj[blk.to].append((blk.frm, blk.bid))

    # ------------------------------------------------------------------
    def find_path(self, start: str, goal: str, cargo_class: str = "clean") -> list[str]:
        """start → goal 최단 경로를 '블록 id 리스트'로 반환합니다.

        cargo_class 가 해당 블록의 forbid 목록에 있으면 그 블록은 지나갈 수 없습니다.
        → 오염 기구 동선 분리를 '소프트웨어만으로' 구현하는 부분입니다.
        경로가 없으면 빈 리스트를 반환합니다.
        """
        if start == goal:
            return []

        prev: dict[str, tuple[str, str]] = {}   # node -> (이전노드, 타고온 블록)
        seen = {start}
        q = deque([start])

        while q:
            cur = q.popleft()
            for nxt, bid in self.adj[cur]:
                if nxt in seen:
                    continue
                if cargo_class in self.blocks[bid].forbid:
                    continue        # 금지 간선(Forbidden Edge)
                seen.add(nxt)
                prev[nxt] = (cur, bid)
                if nxt == goal:
                    return self._rebuild(prev, start, goal)
                q.append(nxt)
        return []

    @staticmethod
    def _rebuild(prev, start, goal) -> list[str]:
        path, cur = [], goal
        while cur != start:
            cur, bid = prev[cur]
            path.append(bid)
        path.reverse()
        return path

    # ------------------------------------------------------------------
    def eta(self, blocks: list[str]) -> float:
        """블록 리스트를 전부 통과하는 데 걸리는 순수 주행시간."""
        return sum(self.blocks[b].traverse_s for b in blocks)

    def siding_of(self, node: str) -> str | None:
        """이 노드에서 대피 명령을 받으면 빠질 지선 노드."""
        return self.sidings.get(node)

    def block_between(self, a: str, b: str) -> str | None:
        for nxt, bid in self.adj.get(a, []):
            if nxt == b:
                return bid
        return None

    def node_name(self, nid: str) -> str:
        return self.nodes.get(nid, {}).get("name", nid)
