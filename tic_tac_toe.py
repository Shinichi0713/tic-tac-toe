import tkinter as tk
from tkinter import font
from typing import NamedTuple
from itertools import cycle
import random
import agent

# 画面反映用クラス
class Player(NamedTuple):
    label: str
    color: str


# 番目の手を表すクラス
class Move(NamedTuple):
    row: int
    col: int
    label: str = ""


BOARD_SIZE = 3
DEFAULT_PLAYERS = (
    Player(label="×", color="blue"),
    Player(label="O", color="red"),
)

# ゲーム画面
class TicTacToeBoard(tk.Tk):
    def __init__(self, game):
        super().__init__()
        self._game = game
        self._create_menu()
        self.title("Tic-Tac-Toe Game")
        self._cells = {}
        self._create_board_display()
        self._create_board_grid()
        # エージェント
        self.agent = agent.Agent(game)

    # ゲーム画面を生成
    def _create_board_display(self):
        display_frame = tk.Frame(master=self)
        display_frame.pack(fill=tk.X)
        self.display = tk.Label(
            master=display_frame,
            text="tic-tac-toe game",
            font=font.Font(size=28, weight="bold"),
        )
        self.display.pack()

    # 番目を生成
    def _create_board_grid(self):
        grid_frame = tk.Frame(master=self)
        grid_frame.pack()
        for row in range(self._game.board_size):
            self.rowconfigure(row, weight=1, minsize=50)
            self.columnconfigure(row, weight=1, minsize=75)
            for col in range(self._game.board_size):
                button = tk.Button(
                    master=grid_frame,
                    text="",
                    font=font.Font(size=36, weight="bold"),
                    fg="black",
                    width=5,
                    height=2,
                    highlightbackground="lightblue",
                )
                self._cells[button] = (row, col)
                button.bind("<ButtonPress-1>", self.play)
                button.grid(
                    row=row,
                    column=col,
                    padx=5,
                    pady=5,
                    sticky="nsew"
                )

    # 実際の操作
    def play(self, event):
        clicked_btn = event.widget
        row, col = self._cells[clicked_btn]
        move = Move(row, col, self._game.current_player.label)
        if self._game.is_valid_move(move):
            self._judge_game(move, clicked_btn)
            # プレーヤの後的の処理
            self._choice_enemy()

    # 敵の手を選択
    def _choice_enemy(self):
        area_movable = self._game.get_area_movable()
        if len(area_movable) == 0:
            return

        # playareaを復元
        playarea = self._game.calculate_play_area()
        # AIの手を選択
        while True:
            # 学習時のラベル対応に合わせるため反転
            playarea = self._game.inverse_playarea(playarea)
            _, ql_ai_input = self.agent.get_ai_input(playarea, 1, mode=1, epsilon=0.0)
            ql_ai_input -= 1
            print(f'AIの手：{ql_ai_input}')
            move_enemy = Move(ql_ai_input // 3, ql_ai_input % 3, self._game.current_player.label)
            if self._game.is_valid_move(move_enemy):
                break
        for button, coordinates in self._cells.items():
            if coordinates == (move_enemy.row, move_enemy.col):
                button_choice = button
        self._judge_game(move_enemy, button_choice)

    # 勝敗判定
    def _judge_game(self, move, clicked_btn):
        if self._game.is_valid_move(move):
            self._game.process_move(move)
            self._update_button(clicked_btn)
            # ゲームが続いている
            if self._game.is_tied():
                self._update_display(msg="Tied game!", color="red")
            # 勝敗が決まった
            elif self._game.has_winner():
                self._highlight_cells()
                msg = f'Player "{self._game.current_player.label}" won!'
                color = self._game.current_player.color
                self._update_display(msg, color)
            # 引き分け
            else:
                msg = "draw"
                self._update_display(msg)

    def _update_button(self, clicked_btn):
        clicked_btn.config(text=self._game.current_player.label)
        clicked_btn.config(fg=self._game.current_player.color)

    def _update_display(self, msg, color="black"):
        self.display["text"] = msg
        self.display["fg"] = color

    def _highlight_cells(self):
        for button, coordinates in self._cells.items():
            if coordinates in self._game.winner_combo:
                button.config(highlightbackground="red")

    def _create_menu(self):
        menu_bar = tk.Menu(master=self)
        self.config(menu=menu_bar)
        file_menu = tk.Menu(master=menu_bar)
        file_menu.add_command(
            label="Play Again",
            command=self.reset_board
        )
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=quit)
        menu_bar.add_cascade(label="File", menu=file_menu)
    
    def reset_board(self):
        self._game.reset_game()
        self._update_display(msg="Ready?")
        for button in self._cells.keys():
            button.config(highlightbackground="lightblue")
            button.config(text="")
            button.config(fg="black")


# tic-tac-toeゲームを進行するクラス
class TicTacToeGame:
    # コンストラクタ
    def __init__(self, players=DEFAULT_PLAYERS, board_size=BOARD_SIZE):
        self.symbol_player = [player.label for player in DEFAULT_PLAYERS]
        self._players = cycle(players)
        self.board_size = board_size
        self.current_player = next(self._players)
        self.winner_combo = []
        self._current_moves = []        # 現状の番目
        self._has_winner = False
        self._winning_combos = []       # 勝敗パターン
        self._setup_board()
    
    def _setup_board(self):
        print("Setting up game rule")
        self._current_moves = [
            [Move(row, col) for col in range(self.board_size)]
            for row in range(self.board_size)
        ]
        self._winning_combos = self._get_winning_combos()
    
    # 勝ちパターンを取得
    def _get_winning_combos(self):
        rows = [
            [(move.row, move.col) for move in row]
            for row in self._current_moves
        ]
        columns = [list(col) for col in zip(*rows)]
        first_diagonal = [row[i] for i, row in enumerate(rows)]
        second_diagonal = [col[j] for j, col in enumerate(reversed(columns))]
        return rows + columns + [first_diagonal, second_diagonal]

    def is_valid_move(self, move):
        row, col = move.row, move.col
        move_was_not_played = self._current_moves[row][col].label == ""
        no_winner = not self._has_winner
        return no_winner and move_was_not_played

    # 入力された手を処理する
    def process_move(self, move):
        row, col = move.row, move.col
        self._current_moves[row][col] = move
        
        for cobo_wining in self._winning_combos:
            result_check = set([self._current_moves[n][m].label for n, m in cobo_wining])
            if len(result_check) == 1 and "" not in result_check:
                self._has_winner = True
                self.winner_combo = cobo_wining
                # print(f'"勝ち："{[self._current_moves[n][m].label for n, m in cobo_wining]}')
                break
        # プレーヤの切り替え
        self.current_player = next(self._players)
        
    # 勝者がいるかどうかを確認
    def has_winner(self):
        return self._has_winner

    # ゲームが引き分けかどうかを確認
    def is_tied(self):
        no_winner = not self._has_winner
        # 番目に隙間があるかどうかを確認する
        moves_variation = set([element.label for col in self._current_moves
            for element in col])
        result_playable = "" in moves_variation
        return no_winner and result_playable

    # リセット処理
    def reset_game(self):
        for row, row_content in enumerate(self._current_moves):
            for col, _ in enumerate(row_content):
                row_content[col] = Move(row, col)
        self._has_winner = False
        self.winner_combo = []

    # エージェントに渡す状態空間に変換
    def get_area_movable(self):
        area_movable = []
        for row in range(3):
            for col in range(3):
                move = Move(row, col, "")
                if self.is_valid_move(move):
                    area_movable.append(move)
        return area_movable

    # エージェントの行動を番目に反映
    def calculate_hand(self, hand_input):
        row = hand_input // 3
        col = hand_input % 3
        move = Move(row, col, self.current_player.label)
        self.process_move(move)

    # 現在の番目から状態空間を取得
    def calculate_play_area(self):
        play_area = []
        for row in range(3):
            for col in range(3):
                if self._current_moves[row][col].label == "":
                    play_area.append(row * 3 + col + 1)
                else:
                    play_area.append(self._current_moves[row][col].label)
        return play_area
    
    # AIの入力用に状態を反転
    def inverse_playarea(self, playarea):
        playarea_inverse = []
        for element in playarea:
            if element == "×":
                playarea_inverse.append("O")
            elif element == "O":
                playarea_inverse.append("×")
            else:
                playarea_inverse.append(element)
        return playarea_inverse

# ゲーム実行
def main():
    game = TicTacToeGame()

    board = TicTacToeBoard(game)
    board.mainloop()

# 検証用関数(メンテナンス用途)
def try_develop():
    game = TicTacToeGame()
    print(game.current_player)
    game.process_move(Move(0, 0, "〇"))
    print(game.current_player)
    game.process_move(Move(0, 1, "〇"))
    print(game.current_player)
    # game.process_move(Move(0, 2, "〇"))
    game.process_move(Move(2, 2, "×"))
    print(game.get_area_movable())
    print(random.choice(game.get_area_movable()))


if __name__ == "__main__":
    main()

