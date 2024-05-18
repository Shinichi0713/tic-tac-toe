# エージェントの学習と行動を定義するクラス
import numpy as np
import random, os
import tic_tac_toe

# 学習済みQテーブルのファイル名
filename_q_table = 'q_table.npy'

class Agent:
    def __init__(self, game):
        self.eta = 0.1  # 学習率
        self.gamma = 0.9  # 時間割引率
        self.initial_epsilon = 0.5  # ε-greedy法の初期値
        self._make_q_table()
        self.game = game

    # Qテーブル作成
    def _make_q_table(self):
        n_columns = 9
        n_rows = 3**9
        path_q_tabel = os.path.join(os.path.dirname(__file__), filename_q_table)
        if os.path.exists(path_q_tabel):
            print('Load Q table')
            self.q_table = np.load(path_q_tabel)
        else:
            print('Create Q table')
            self.q_table = np.zeros((n_rows, n_columns))
    
    # Qテーブル更新
    def q_learning(self, play_area, ai_input, reward, play_area_next, end_flg):
        # 行番号取得
        row_index = self._find_q_row(play_area)
        row_index_next = self._find_q_row(play_area_next)
        column_index = ai_input - 1
        # 勝負がついた場合
        if end_flg == 1:
            self.q_table[row_index, column_index] = \
                self.q_table[row_index, column_index] + self.eta \
                * (reward - self.q_table[row_index, column_index])
        # まだ続いている場合
        else:
            self.q_table[row_index, column_index] = \
                self.q_table[row_index, column_index] + self.eta \
                * (reward + self.gamma * np.nanmax(self.q_table[row_index_next, :]) \
                - self.q_table[row_index, column_index])
    
    # Qテーブルの行番号取得
    def _find_q_row(self, play_area):
        row_index = 0
        for index in range(len(play_area)):
            if play_area[index] == self.game.symbol_player[0]:
                coef = 1
            elif play_area[index] == self.game.symbol_player[1]:
                coef = 2
            else:
                coef = 0
            row_index += (3 ** index) * coef
        return row_index
    
    # greedy法による行動選択
    def choice_q_action(self, play_area, choosable_area, epsilon):
        ai_input = None
        # esilonの確率でランダムな選択をする
        if np.random.rand() < epsilon:
            ai_input = int(random.choice(choosable_area))
        # Qテーブルに従い行動を選択する
        else:
            row_index = self._find_q_row(play_area)
            first_choice_flg = 1
            q_max = -9999
            for choice in choosable_area:
                if first_choice_flg == 1:
                    first_choice_flg = 0
                    q_max = self.q_table[row_index, int(choice)-1]
                else:
                    if q_max < self.q_table[row_index, int(choice)-1]:
                        q_max = self.q_table[row_index, int(choice)-1]
            # Q値の最大値を持つ行動のインデックスを格納
            actions_candidate = [choice for choice in choosable_area if q_max == self.q_table[row_index, int(choice)-1]]
            ai_input = int(random.choice(actions_candidate))
        # AI入力が得られない場合、ランダムに選択
        if ai_input is None:
            ai_input = int(random.choice(choosable_area))
        return ai_input
    
    # AIの行動を取得、行動ラベル出力
    def get_ai_input(self, play_area, first_inputter, mode=0, epsilon=None):
        choosable_area = [str(area) for area in play_area if type(area) is int]
        # ランダム行動
        if mode == 0:
            ai_input = int(random.choice(choosable_area))
        # Qテーブルベース行動
        elif mode == 1:
            ai_input = self.choice_q_action(play_area, choosable_area, epsilon)
        
        # 学習時は必要、対戦段階では不要
        if first_inputter == 1:
            play_area[play_area.index(ai_input)] = self.game.symbol_player[0]
        elif first_inputter == 2:
            play_area[play_area.index(ai_input)] = self.game.symbol_player[1]
        
        return play_area, ai_input

    # Q学習実行
    def train_qtable(self, first_inputter, epsilon=0):
        # Q学習退避用
        ql_input_list = []
        # 状態量の変数
        play_area_list = []

        play_area = list(range(1, 10))
        inputter_count = first_inputter
        end_flg = 0
        ql_flg = 0
        reward = 0
        result = 0  # 0:引き分け, 1:AI(1)勝利, 2:AI(2)勝利
        # 1ゲームの実行
        while True:
            # Q学習状態量履歴
            play_area_tmp = play_area.copy()
            play_area_list.append(play_area_tmp)
            # Q学習実行フラグ
            ql_flg = 0
            # AI(Q学習)の手番
            if (inputter_count % 2) == 0:
                # QL AI入力
                play_area, ql_ai_input = self.get_ai_input(play_area, 
                                                    first_inputter,
                                                    mode=1, 
                                                    epsilon=epsilon)
                self.game.calculate_hand(ql_ai_input - 1)
                winner = self.game.has_winner()
                end_flg = not self.game.is_tied() or winner
                # Q学習退避用
                ql_input_list.append(ql_ai_input)            
                # AIが勝利した場合
                if winner:
                    reward = 4
                    ql_flg = 1
                    result = 1
                play_area_before = play_area_list[-1]
                ql_ai_input_before = ql_input_list[-1]
            # AI(ランダム)の手番
            elif (inputter_count % 2) == 1:
                play_area, random_ai_input = self.get_ai_input(play_area, 
                                                        first_inputter+1, 
                                                        mode=0)
                self.game.calculate_hand(random_ai_input - 1)
                winner = self.game.has_winner()
                end_flg = not self.game.is_tied() or winner
                # AIが負けた場合
                if winner:
                    reward = -1
                    ql_flg = 1
                    result = 2
            # Q学習実行
            if ql_flg == 1:
                ql_ai_input_before = ql_input_list[-1]
                self.q_learning(play_area_before, ql_ai_input_before,
                                reward, play_area, end_flg)
            # ゲーム終了
            if end_flg:
                # ゲーム初期化
                self.game.reset_game()
                break
            inputter_count += 1
        return winner, self.q_table, result
    

# エージェントを学習
def train_agent():
    game = tic_tac_toe.TicTacToeGame()
    agent = Agent(game=game)
    epoch = 100000
    epsiron = 0.0   # ε-greedy法の閾値
    interval = epoch // 10
    count_ai_win = 0
    count_ai_lose = 0
    for i in range(epoch):
        winner, q_table, result = agent.train_qtable(1, epsiron)
        if i % interval == 0:
            print(f'Epoch: {i}, Winner: {winner}')
        if winner:
            epsiron = epsiron * 0.9
        if result == 1:
            count_ai_win += 1
        elif result == 2:
            count_ai_lose += 1
    # Qテーブル保存
    np.save(filename_q_table, q_table)
    print('Save Q table')
    print(q_table.sum(), epsiron, count_ai_win, count_ai_lose)
    return winner, q_table

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    train_agent()