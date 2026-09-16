from motion_toolbox.graph import lazy_shortest_path


def test_bounded_proposals_do_not_claim_disconnection_or_accept_bad_edge():
    layers = [[[0.]], [[0.], [1.], [2.]], [[0.]]]
    calls = []
    def valid(i,a,b):
        calls.append((i,a,b))
        return i!=1 or b==2
    bounded = lazy_shortest_path(layers,edge_valid=valid,lazy_rounds=1,
                                 exhaustive_fallback=False)
    assert not bounded.configurations
    assert bounded.failure_layer==1
    assert len(calls)==2
    exact = lazy_shortest_path(layers,edge_valid=valid,lazy_rounds=1)
    assert exact.indices==[0,2,0]


def test_bounded_success_checks_every_selected_edge():
    seen = []
    result = lazy_shortest_path([[[0.]],[[.1]],[[.2]]],
        edge_valid=lambda i,a,b: seen.append((i,a,b)) or True,
        exhaustive_fallback=False)
    assert result.indices==[0,0,0]
    assert seen==[(1,0,0),(2,0,0)]


def test_cheap_disconnection_does_not_trigger_expensive_fallback():
    result = lazy_shortest_path([[[0.]],[[10.]]],max_step=1.,
        edge_valid=lambda *args: (_ for _ in ()).throw(AssertionError()),
        exhaustive_fallback=False)
    assert not result.configurations
    assert result.failure_layer==1
