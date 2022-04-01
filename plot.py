import json
import math

import plotly.graph_objects as go
import dash
from dash import dcc, html, Input, Output
import joint_defs
from actuator import JointData


def plot_all() -> go.Figure():
    fig = go.Figure()
    fig.add_scatter3d(
        name="",
        visible=True,
        showlegend=False,
        opacity=0,
        hoverinfo='none',
        x=[-700, 700],
        y=[-700, 700],
        z=[-1400, 0]
    )
    fig.update_layout(
        width=800,
        height=600,
        scene=dict(
            # camera=dict(
            #     up=dict(x=0, y=0, z=0),
            # ),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1)
        ),
        clickmode='event+select'
    )
    # Plot all origins before all actuators so it's easy to identify the clicked origin by curveNumber
    for j in joint_defs.ALL_JOINTS:
        j.plot_origins_plotly(fig)
    for j in joint_defs.ALL_JOINTS:
        j.plot_actuators_plotly(fig)
    return fig


selected_joint: JointData = None
angle_slider_params = dict(value=0, marks=None, updatemode='drag',
                           tooltip={'placement': 'bottom', 'always_visible': True})
fig: go.Figure = plot_all()

app = dash.Dash()
app.layout = html.Div([
    dcc.Graph(id='graph', figure=fig),
    html.Div([html.Label('Selected Joint: None')], id='selected-joint-label'),
    html.Div([
        html.Label('Yaw'),
        dcc.Slider(-90, 90, 1, id='yaw-slider', **angle_slider_params)
    ], id='yaw-slider-div'),
    html.Div([
        html.Label('Pitch'),
        dcc.Slider(-90, 90, 1, id='pitch-slider', **angle_slider_params)
    ], id='pitch-slider-div'),
    html.Div([
        html.Label('Roll'),
        dcc.Slider(-90, 90, 1, id='roll-slider', **angle_slider_params)
    ], id='roll-slider-div')
])


@app.callback(
    Output('graph', 'figure'),
    [
        Input('selected-joint', 'value'),
        Input('yaw-slider', 'value'),
        Input('pitch-slider', 'value'),
        Input('roll-slider', 'value')
    ]
)
def update_graph(yaw_value, pitch_value, roll_value):
    if selected_joint:
        app.logger.info(f"Setting value of {selected_joint.name}: {yaw_value}, {pitch_value}, {roll_value}")
        selected_joint.angles[1] = math.radians(yaw_value)
        selected_joint.angles[0] = math.radians(pitch_value)
        selected_joint.angles[2] = math.radians(roll_value)

    return plot_all()


@app.callback(
    [
        Output('selected-joint')
        Output('selected-joint-label', 'children')
    ],
    Input('graph', 'clickData')
)
def update_click(click_data):
    selected_joint_label: html.Label = None
    # app.logger.info(f"{json.dumps(click_data, indent=4)}")
    if click_data:
        selected_origin_curve_num = click_data['points'][0]['curveNumber']
        selected_joint = joint_defs.ALL_JOINTS[selected_origin_curve_num - 1]
        selected_joint_label = html.Label(f"Selected Joint: {selected_joint.name}")
    else:
        selected_joint_label = html.Label('Selected Joint: None')

    return selected_joint_label


if __name__ == '__main__':
    app.run_server(debug=True)
