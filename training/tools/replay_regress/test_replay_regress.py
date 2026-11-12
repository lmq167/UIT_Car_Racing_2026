"""Kiểm so sánh ý định và lệnh, kể cả khi cùng frame được ghi nhiều lần."""
import unittest
from replay_regress import FIELDS,differences
from mask_probe import corridor
import numpy as np


class RegressionTest(unittest.TestCase):
    def sample(self):
        return {k:'0' for k in FIELDS}

    def test_intent_changes_without_command_change(self):
        old=self.sample();new=dict(old);new['turn_dir']='1'
        self.assertEqual(differences(old,new),['turn_dir'])

    def test_numeric_text_and_tolerance(self):
        old=self.sample();new=dict(old);new['cmd_deg']='0.0000001';new['cmd_cms']='0.0'
        self.assertEqual(differences(old,new),[])
        new['cmd_deg']='0.001'
        self.assertEqual(differences(old,new),['cmd_deg'])

    def test_empty_mask_has_no_invented_lookahead(self):
        p={'center_offset_frac':0,'corridor_bottom_frac':.95,'corridor_top_frac':.62,'corridor_rows':8,
           'lane1_width_frac':[.57,.17],'two_lane_ratio':1.5,'max_jump_frac':.08,'corridor_conf_min':.4,
           'min_valid_rows':1,'lookahead_frac':.75}
        mask=np.zeros((192,320),dtype=np.uint8)
        self.assertIsNone(corridor(mask,p)[0]);self.assertIsNone(corridor(mask,p,True)[0])


if __name__=='__main__':unittest.main()
