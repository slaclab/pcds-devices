import pytest
import numpy as np
from ophyd.sim import make_fake_device
from pcdsdevices.targets import (XYGridStage, convert_to_physical,
                                 get_unit_meshgrid, mesh_interpolation,
                                 snake_grid_list)
from pcdsdevices.sim import FastMotor
import yaml


@pytest.fixture(scope='function')
def sample_file(tmp_path):
    path = tmp_path / "sub"
    path.mkdir()
    sample_file = path / "test_sample.yml"
    sample_file.write_text("""
test_sample:
  time_created: '2021-01-22 14:29:29.681059'
  top_left:
  - -20.59374999999996
  - 26.41445312499999
  top_right:
  - -19.838671874999946
  - 26.408203124999986
  bottom_right:
  - -19.426171874999945
  - 51.39570312500023
  bottom_left:
  - -20.176171875000115
  - 51.414453125000215
  M: 101
  N: 4
  coefficients:
  - -20.59374999999996
  - 0.7550781250000149
  - 0.4175781249998458
  - -0.005078124999844391
  - 26.41445312499999
  - -0.006250000000004974
  - 25.000000000000224
  - -0.012499999999977973
  xx:
  - pos: -20.59374999999996
    status: true
  - pos: -20.342057291666624
    status: true
  - pos: -20.090364583333283
    status: true
  - pos: -19.838671874999946
    status: true
  - pos: -19.834546874999948
    status: false
  - pos: -20.08622265624995
    status: false
  - pos: -20.33789843749996
    status: false
  - pos: -20.589574218749963
    status: false
  yy:
  - pos: 26.41445312499999
    status: true
  - pos: 26.412369791666656
    status: true
  - pos: 26.41028645833332
    status: true
  - pos: 26.408203124999986
    status: true
  - pos: 26.664453124999994
    status: false
  - pos: 26.66232812499999
    status: false
  - pos: 26.660203124999992
    status: false
  - pos: 26.65807812499999
    status: false
    """)
    return sample_file


@pytest.fixture(scope='function')
def fake_grid_stage(sample_file):
    FakeGridStage = make_fake_device(XYGridStage)
    x_motor = FastMotor()
    y_motor = FastMotor()
    grid = FakeGridStage(
        x_motor=x_motor,
        y_motor=y_motor, m_points=5, n_points=5,
        path=sample_file.parent)
    grid._current_sample = 'current_sample'
    # dummy coeffs
    # a_coeffs = [0.0, 4.0, 0.0, 0.0]
    # b_coeffs = [0.0, 0.0, 4.0, 0.0]
    grid.coefficients = [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0]
    return grid


def test_get_samples(fake_grid_stage, sample_file):
    xy = fake_grid_stage
    res = xy.get_samples()
    # not existing files yet
    assert res == ['test_sample']
    xy.save_grid(sample_name='sample1', path=sample_file.parent)
    xy.save_grid(sample_name='sample2', path=sample_file.parent)
    # test all mapped samples
    res = xy.get_samples(path=sample_file.parent)
    for ff in res:
        assert ff in ['sample1', 'sample2', 'test_sample']
    assert len(res) == 3
    with pytest.raises(Exception):
        xy.get_samples(path='bad_file')


def test_get_set_current_sample(fake_grid_stage):
    xy = fake_grid_stage
    current_sample = xy.current_sample
    assert current_sample == 'current_sample'


def test_get_sample_data(fake_grid_stage, sample_file):
    xy = fake_grid_stage
    res = xy.get_sample_data('sample1', path=sample_file)
    assert res == {}
    # test test_sample in the file:
    res = xy.get_sample_data('test_sample')
    assert res['M'] == 101
    assert res["N"] == 4
    xy.save_grid(sample_name='sample2', path=sample_file.parent)
    res = xy.get_sample_data('sample2')
    coeffs = [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0]
    assert res['coefficients'] == coeffs
    # return empty dictionary if could not find the sample
    res = xy.get_sample_data("non_sample", path=sample_file)
    assert res == {}
    # raise an exception if could not load file
    with pytest.raises(Exception):
        xy.get_sample_data("non_sample", path='no_file')


def test_get_sample_map_info(fake_grid_stage, sample_file):
    xy = fake_grid_stage
    xy.save_grid(sample_name='sample2', path=sample_file.parent)
    coeffs = [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0]
    # test get_sample_map_info - dummy values
    res = xy.get_sample_map_info('sample2')
    assert res == (5, 5, coeffs)


def test_mapping_points(fake_grid_stage):
    # test grid of 5 rows by 5 columns
    top_left, top_right, bottom_right, bottom_left = ((0, 0), (4, 0), (4, 4),
                                                      (0, 4))
    x, y = fake_grid_stage.map_points(snake_like=False, top_left=top_left,
                                      top_right=top_right,
                                      bottom_right=bottom_right,
                                      bottom_left=bottom_left,
                                      m_rows=5, n_columns=5)
    expected_x_points = [0.0, 1.0, 2.0, 3.0, 4.0,
                         0.0, 1.0, 2.0, 3.0, 4.0,
                         0.0, 1.0, 2.0, 3.0, 4.0,
                         0.0, 1.0, 2.0, 3.0, 4.0,
                         0.0, 1.0, 2.0, 3.0, 4.0]

    expected_y_points = [0.0, 0.0, 0.0, 0.0, 0.0,
                         1.0, 1.0, 1.0, 1.0, 1.0,
                         2.0, 2.0, 2.0, 2.0, 2.0,
                         3.0, 3.0, 3.0, 3.0, 3.0,
                         4.0, 4.0, 4.0, 4.0, 4.0]
    assert expected_x_points == x
    assert expected_y_points == y

    # test grid of 3 rows by 5 columns
    top_left, top_right, bottom_right, bottom_left = ((0, 0), (2, 0), (2, 4),
                                                      (0, 4))
    x, y = fake_grid_stage.map_points(snake_like=False, top_left=top_left,
                                      top_right=top_right,
                                      bottom_right=bottom_right,
                                      bottom_left=bottom_left,
                                      m_rows=5, n_columns=3)
    expected_x_points = [0.0, 1.0, 2.0,
                         0.0, 1.0, 2.0,
                         0.0, 1.0, 2.0,
                         0.0, 1.0, 2.0,
                         0.0, 1.0, 2.0]

    expected_y_points = [0.0, 0.0, 0.0,
                         1.0, 1.0, 1.0,
                         2.0, 2.0, 2.0,
                         3.0, 3.0, 3.0,
                         4.0, 4.0, 4.0]
    assert expected_x_points == x
    assert expected_y_points == y

    # test 5 by 5 grid with slope of -0.25
    top_left, top_right, bottom_right, bottom_left = ((0, 0), (4, -1), (5, 3),
                                                      (1, 4))
    x, y = fake_grid_stage.map_points(snake_like=False, top_left=top_left,
                                      top_right=top_right,
                                      bottom_right=bottom_right,
                                      bottom_left=bottom_left,
                                      m_rows=5, n_columns=5)
    expected_x_points = [0.0, 1.0, 2.0, 3.0, 4.0,
                         0.25, 1.25, 2.25, 3.25, 4.25,
                         0.50, 1.50, 2.50, 3.50, 4.50,
                         0.75, 1.75, 2.75, 3.75, 4.75,
                         1.0, 2.0, 3.0, 4.0, 5.0]

    expected_y_points = [0.0, -0.25, -0.50, -0.75, -1,
                         1.0, 0.75, 0.50, 0.25, 0,
                         2.0, 1.75, 1.50, 1.25, 1,
                         3.0, 2.75, 2.50, 2.25, 2,
                         4.0, 3.75, 3.50, 3.25, 3]
    assert expected_x_points == x
    assert expected_y_points == y


def test_mapping_points_snake_like(fake_grid_stage):
    # test 5 by 5 grid with slope of -0.25
    top_left, top_right, bottom_right, bottom_left = ((0, 0), (4, -1), (5, 3),
                                                      (1, 4))
    x, y = fake_grid_stage.map_points(snake_like=True, top_left=top_left,
                                      top_right=top_right,
                                      bottom_right=bottom_right,
                                      bottom_left=bottom_left,
                                      m_rows=5, n_columns=5)
    expected_x_points = [0.0, 1.0, 2.0, 3.0, 4.0,
                         4.25, 3.25, 2.25, 1.25, 0.25,
                         0.50, 1.50, 2.50, 3.50, 4.50,
                         4.75, 3.75, 2.75, 1.75, 0.75,
                         1.0, 2.0, 3.0, 4.0, 5.0]
    expected_y_points = [0.0, -0.25, -0.50, -0.75, -1,
                         0, 0.25, 0.50, 0.75, 1.0,
                         2.0, 1.75, 1.50, 1.25, 1.0,
                         2.0, 2.25, 2.50, 2.75, 3.0,
                         4.0, 3.75, 3.50, 3.25, 3.0]
    assert expected_x_points == x
    assert expected_y_points == y


def test_compute_mapped_point(fake_grid_stage, sample_file):
    stage = fake_grid_stage
    stage.save_grid(sample_name='test_sample', path=sample_file.parent)

    x, y = stage.compute_mapped_point(
        m_row=1, n_column=1, sample_name='test_sample',
        path=sample_file.parent)
    # first row, first column
    assert (x, y) == (0, 0)
    x, y = stage.compute_mapped_point(
        m_row=1, n_column=2, sample_name='test_sample',
        path=sample_file.parent)
    # first row, second column
    assert (x, y) == (1.0, 0)

    stage.load('test_sample')
    assert stage.current_sample == 'test_sample'
    x, y = stage.compute_mapped_point(1, 1)
    # first row, first column
    assert (x, y) == (0, 0)
    x, y = stage.compute_mapped_point(1, 2)
    # first row, second column
    assert (x, y) == (1.0, 0)

    with pytest.raises(IndexError):
        stage.compute_mapped_point(0, 0, 'test_sample',)
    with pytest.raises(IndexError):
        stage.compute_mapped_point(6, 0, 'test_sample')


def test_move_to_sample(fake_grid_stage, sample_file):
    # position of x and y shouold be 0, 0
    stage = fake_grid_stage
    stage.current_sample = 'test_sample'
    assert stage.x.position == 0
    assert stage.y.position == 0
    stage.move_to_sample(3, 1)
    assert stage.x.position == 0.0
    assert stage.y.position == 2.0


def test_move_to(fake_grid_stage):
    stage = fake_grid_stage
    assert stage.x.position == 0
    assert stage.y.position == 0
    stage.move_to('test_sample', 3, 1)
    assert stage.x.position == 0.0
    assert stage.y.position == 2.0


def test_load(fake_grid_stage, sample_file):
    stage = fake_grid_stage
    stage.save_grid(sample_name='current_sample', path=sample_file.parent)
    stage.load('current_sample')
    assert stage.current_sample == 'current_sample'


def test_set_m_n_points(fake_grid_stage):
    stage = fake_grid_stage
    assert stage.m_n_points == (5, 5)
    stage.m_n_points = 10, 23
    assert stage.m_n_points == (10, 23)
    with pytest.raises(Exception):
        stage.m_n_points = 23


def test_mesh_interpolation():
    a_coeffs, b_coeffs = mesh_interpolation((0, 0), (4, 0), (4, 4),
                                            (0, 4))
    assert np.isclose(a_coeffs, [0.0, 4.0,  0.0, 0.0]).all()
    assert np.isclose(b_coeffs, [0.0, 0.0, 4.0, 0.0]).all()
    # test with slope -0.25
    a_coeffs, b_coeffs = mesh_interpolation((0, 0), (4, -1), (5, 3),
                                            (1, 4))
    assert np.isclose(a_coeffs, [0.0, 4.0,  1.0, 0.0]).all()
    assert np.isclose(b_coeffs, [0.0, -1.0, 4.0, 0.0]).all()


def test_get_unit_meshgrid():
    grid = get_unit_meshgrid(m_rows=5, n_columns=5)
    xx_expected = [[0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0]]
    yy_expected = [[0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.25, 0.25, 0.25, 0.25, 0.25],
                   [0.5, 0.5, 0.5, 0.5, 0.5],
                   [0.75, 0.75, 0.75, 0.75, 0.75],
                   [1.0, 1.0, 1.0, 1.0, 1.0]]

    assert grid[0].tolist() == xx_expected
    assert grid[1].tolist() == yy_expected


def test_convert_to_physical():
    a_coeffs = [0.0, 4.0, 0.0, 0.0]
    b_coeffs = [0.0, 0.0, 4.0, 0.0]
    # based on these logical values:
    # xx_expected = [[0, 0.25, 0.5, 0.75, 1.0],
    #                [0, 0.25, 0.5, 0.75, 1.0],
    #                [0, 0.25, 0.5, 0.75, 1.0],
    #                [0, 0.25, 0.5, 0.75, 1.0],
    #                [0, 0.25, 0.5, 0.75, 1.0]]
    # yy_expected = [[0.0, 0.0, 0.0, 0.0, 0.0],
    #                [0.25, 0.25, 0.25, 0.25, 0.25],
    #                [0.5, 0.5, 0.5, 0.5, 0.5],
    #                [0.75, 0.75, 0.75, 0.75, 0.75],
    #                [1.0, 1.0, 1.0, 1.0, 1.0]]
    # should get these results:
    # expected_x_points = [0.0, 1.0, 2.0, 3.0, 4.0,
    #                      0.0, 1.0, 2.0, 3.0, 4.0,
    #                      0.0, 1.0, 2.0, 3.0, 4.0,
    #                      0.0, 1.0, 2.0, 3.0, 4.0,
    #                      0.0, 1.0, 2.0, 3.0, 4.0]

    # expected_y_points = [0.0, 0.0, 0.0, 0.0, 0.0,
    #                      1.0, 1.0, 1.0, 1.0, 1.0,
    #                      2.0, 2.0, 2.0, 2.0, 2.0,
    #                      3.0, 3.0, 3.0, 3.0, 3.0,
    #                      4.0, 4.0, 4.0, 4.0, 4.0]
    x, y = convert_to_physical(a_coeffs, b_coeffs, 0, 0)
    # should be 0, 0
    assert (x, y) == (0.0, 0.0)
    x, y = convert_to_physical(a_coeffs, b_coeffs, 0.75, 0.75)
    # should be 3, 3
    assert (x, y) == (3.0, 3.0)
    x, y = convert_to_physical(a_coeffs, b_coeffs, 0.5, 0.0)
    # should be 2, 0
    assert (x, y) == (2.0, 0.0)


def test_snake_like_list():
    xx = np.array([[0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0],
                   [0, 0.25, 0.5, 0.75, 1.0]])
    yy = np.array([[0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.25, 0.25, 0.25, 0.25, 0.25],
                   [0.5, 0.5, 0.5, 0.5, 0.5],
                   [0.75, 0.75, 0.75, 0.75, 0.75],
                   [1.0, 1.0, 1.0, 1.0, 1.0]])

    # expected values:
    xx_expected = [0, 0.25, 0.5, 0.75, 1.0,
                   1.0, 0.75, 0.5, 0.25, 0,
                   0, 0.25, 0.5, 0.75, 1.0,
                   1.0, 0.75, 0.5, 0.25, 0,
                   0, 0.25, 0.5, 0.75, 1.0]
    yy_expected = [0.0, 0.0, 0.0, 0.0, 0.0,
                   0.25, 0.25, 0.25, 0.25, 0.25,
                   0.5, 0.5, 0.5, 0.5, 0.5,
                   0.75, 0.75, 0.75, 0.75, 0.75,
                   1.0, 1.0, 1.0, 1.0, 1.0]

    xx_res = snake_grid_list(xx)
    # the y values are basically stying the same
    # so there is no need to even run thm through this function
    yy_res = snake_grid_list(yy)
    assert xx_res == xx_expected
    assert yy_res == yy_expected


def test_reset_status(fake_grid_stage, sample_file):
    stage = fake_grid_stage
    origin_info = stage.get_sample_data('test_sample', sample_file)
    xx = origin_info.get('xx')
    yy = origin_info.get('yy')
    x_statuses = [x['status'] for x in xx]
    y_statuses = [y['status'] for y in yy]
    x_pos_expected = [x['pos'] for x in xx]
    y_pos_expected = [y['pos'] for y in yy]
    yaml_dict_original = []
    with open(sample_file) as f:
        yaml_dict = yaml.safe_load(f)
    # current statuses
    current_statuses = [True, True, True, True, False, False, False, False]
    assert x_statuses == current_statuses == y_statuses
    stage.reset_statuses('test_sample', sample_file)
    new_statuses = [False, False, False, False, False, False, False, False]
    # make sure the points have not been changed
    info = stage.get_sample_data('test_sample', sample_file)
    xx = info.get('xx')
    yy = info.get('yy')
    x_statuses = [x['status'] for x in xx]
    y_statuses = [y['status'] for y in yy]
    x_pos = [x['pos'] for x in xx]
    y_pos = [y['pos'] for y in yy]
    assert new_statuses == x_statuses == y_statuses

    # make sure the points are still the same:
    assert x_pos == x_pos_expected
    assert y_pos == y_pos_expected
    with open(sample_file) as f:
        yaml_dict = yaml.safe_load(f)
        assert yaml_dict_original != yaml_dict
        assert len(yaml_dict) == 1
        assert (yaml_dict['test_sample']['time_created'] ==
                origin_info.get('time_created'))
        assert (yaml_dict['test_sample']['coefficients'] ==
                origin_info.get('coefficients'))
        assert (yaml_dict['test_sample']['M'] ==
                origin_info.get('M'))
        assert (yaml_dict['test_sample']['N'] ==
                origin_info.get('N'))
        assert len(yaml_dict['test_sample']) == 10


def test_is_target_shot(fake_grid_stage, sample_file):
    stage = fake_grid_stage
    stage.load('test_sample')
    are_shot = []
    for row in range(1, 3):
        for column in range(1, 5):
            is_shot = stage.is_target_shot(row, column)
            are_shot.append(is_shot)
    assert are_shot == [True, True, True, True,
                        False, False, False, False]


def test_set_status(fake_grid_stage, sample_file):
    stage = fake_grid_stage
    stage.load('test_sample')
    stage.set_status(1, 1, False, 'test_sample')
    info = stage.get_sample_data('test_sample')
    expceted = [False, True, True, True,
                False, False, False, False]
    xx = info['xx']
    res = []
    for x in xx:
        res.append(x['status'])

    assert expceted == res

    with pytest.raises(IndexError):
        stage.set_status(1, 5, False, 'test_sample')
