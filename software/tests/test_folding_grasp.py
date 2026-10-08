from carton.folding_grasp import opposing_faces


def contact(dot, force=1.):
    return dict(normal_dot=dot, normal_force_N=force)


def test_two_jaws_touching_an_edge_do_not_prove_a_pinch():
    assert not opposing_faces([contact(.12)], [contact(-.99)])
    assert not opposing_faces([contact(.99)], [contact(.99)])
    assert not opposing_faces([contact(.99, 0.)], [contact(-.99)])
    assert not opposing_faces([], [contact(-.99)])
    assert opposing_faces([contact(.99)], [contact(-.99)])
