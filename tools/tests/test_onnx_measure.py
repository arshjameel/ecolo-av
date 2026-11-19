"""measure onnx's verdicts, without ONNX Runtime or a model."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import onnx_measure as om  # noqa: E402


def test_reads_core_mls_share_of_the_graph():
    log = ('... GetCapability, number of partitions supported by CoreML: 2 number of nodes '
           'in the graph: 808 number of nodes supported by CoreML: 802\n')
    assert om.coverage_from_log(log) == {'partitions': 2, 'nodes': 808, 'taken': 802}
    assert om.coverage_from_log('nothing here') is None


def test_counts_what_fell_back_to_the_cpu():
    trace = [{'cat': 'Node', 'name': 'a_kernel_time',
              'args': {'provider': 'CPUExecutionProvider', 'op_name': 'Reshape'}},
             {'cat': 'Node', 'name': 'b_kernel_time',
              'args': {'provider': 'CoreMLExecutionProvider', 'op_name': 'CoreML_0'}},
             {'cat': 'Session', 'name': 'model_run', 'args': {}}]
    assert om.cpu_ops(trace) == {'Reshape': 1}


def test_verdicts():
    whole = {'provider': 'coreml', 'answers_ok': True,
             'coverage': {'partitions': 1, 'nodes': 5, 'taken': 5}}
    cpu = {'provider': 'cpu', 'answers_ok': True, 'coverage': None}
    assert om.verdict([cpu, whole]) == 'PASS'
    part = {**whole, 'coverage': {'partitions': 2, 'nodes': 5, 'taken': 4}}
    assert om.verdict([cpu, part]) == 'WARN'
    assert om.verdict([cpu, {**whole, 'answers_ok': False}]) == 'FAIL'
    assert om.verdict([cpu, {'provider': 'coreml', 'error': 'no'}]) == 'FAIL'
