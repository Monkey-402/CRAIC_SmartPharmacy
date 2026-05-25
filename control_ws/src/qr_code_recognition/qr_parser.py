#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def parse_qr(qr_list):
    has_a = False
    has_b = False
    has_c = False
    delivery_slot = 0

    for q in qr_list:
        q_lower = q.lower()

        if "a" in q_lower:
            has_a = True
        if "b" in q_lower:
            has_b = True
        if "c" in q_lower:
            has_c = True

        if "blood" in q_lower:
            delivery_slot = 1
        elif "fluid" in q_lower:
            delivery_slot = 2
        elif "immune" in q_lower:
            delivery_slot = 3
        elif "hormone" in q_lower:
            delivery_slot = 4

    sample_count = int(has_a) + int(has_b) + int(has_c)
    return has_a, has_b, has_c, delivery_slot, sample_count
