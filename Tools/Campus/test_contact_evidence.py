from contact_evidence import classified_contacts


def manifest():
    return dict(boxes=[dict(id='ramp',kind='ramp'),dict(id='pallet',kind='pallet')])


def event(identifier,kind='contact-enter'):
    return dict(type=kind,id=identifier)


def test_ramp_contacts_remain_visible_but_are_not_obstacle_collisions():
    d=classified_contacts(manifest(),dict(obstacles=[]),[event('ramp'),event('ramp','contact-exit')])
    assert d['terrainContactCallbacks']==1 and d['noProhibitedObstacleContacts']


def test_pallet_contact_is_prohibited_even_in_a_ramp_scenario():
    d=classified_contacts(manifest(),dict(obstacles=[]),[event('ramp'),event('pallet')])
    assert d['obstacleContactCallbacks']==1 and not d['noProhibitedObstacleContacts']


def test_unknown_contact_and_scenario_obstacle_are_not_ignored():
    d=classified_contacts(manifest(),dict(obstacles=[dict(id='cart',kind='tote-cart')]),[event('cart'),event('missing')])
    assert d['obstacleContactCallbacks']==1 and d['unknownContactCallbacks']==1
    assert not d['noProhibitedObstacleContacts']
