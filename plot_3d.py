import json
import logging
import math

import plotly.graph_objects as go
import dash
from dash import dcc, html, Input, Output, State
import joint_defs

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
    # fig.update_scenes(xaxis_visible=False, yaxis_visible=False, zaxis_visible=False)
    # Plot all origins before all actuators so it's easy to identify the clicked origin by curveNumber
    for j in joint_defs.ALL_JOINTS:
        j.plot_origins_plotly(fig)
    for j in joint_defs.ALL_JOINTS:
        j.plot_actuators_plotly(fig)
    return fig


angle_slider_params = dict(value=0, marks=None, updatemode='drag',
                           tooltip={'placement': 'bottom', 'always_visible': True})
fig: go.Figure = plot_all()

app = dash.Dash()
app.logger.setLevel(level=logging.DEBUG)
app.layout = html.Div([
    dcc.Graph(id='graph', figure=fig),
    dcc.Store(id='selected-joint', ),
    html.Div([html.Label('Selected Joint: None')], id='selected-joint-label'),
    # html.Div([
    #     html.Label('Yaw'),
    #     dcc.Slider(-90, 90, 1, id='yaw-slider', **angle_slider_params)
    # ], id='yaw-slider-div'),
    # html.Div([
    #     html.Label('Pitch'),
    #     dcc.Slider(-90, 90, 1, id='pitch-slider', **angle_slider_params)
    # ], id='pitch-slider-div'),
    # html.Div([
    #     html.Label('Roll'),
    #     dcc.Slider(-90, 90, 1, id='roll-slider', **angle_slider_params)
    # ], id='roll-slider-div')
    html.Div([
        dcc.Input(id='yaw-input', type='number', placeholder='Yaw'),
        dcc.Input(id='pitch-input', type='number', placeholder='Pitch'),
        dcc.Input(id='roll-input', type='number', placeholder='Roll'),
    ], id='angle-input-div'),
    html.Div([
        dcc.Input(id='torque-yaw-input', type='number', placeholder='Torque Yaw'),
        dcc.Input(id='torque-pitch-input', type='number', placeholder='Torque Pitch'),
        dcc.Input(id='torque-roll-input', type='number', placeholder='Torque Roll'),
    ], id='torque-input-div'),
    # TODO force input
])


@app.callback(
    [
        Output('graph', 'figure'),
        Output('selected-joint-label', 'children')
    ],
    [
        Input('yaw-slider', 'value'),
        Input('pitch-slider', 'value'),
        Input('roll-slider', 'value'),
        Input('graph', 'clickData')
    ],
    State('selected-joint', 'data')
)
def update_graph(yaw_value, pitch_value, roll_value, click_data, selected_joint):
    selected_joint_label = html.Label(f"Selected Joint: None")
    # app.logger.info(f"{json.dumps(click_data, indent=4)}")
    if click_data:
        joint_idx = click_data['points'][0]['curveNumber'] - 1
        selected_joint = joint_defs.ALL_JOINTS[joint_idx]
        selected_joint_label = html.Label(f"Selected Joint: {selected_joint.name}")
        selected_joint = joint_idx

    if selected_joint is not None:
        joint = joint_defs.ALL_JOINTS[selected_joint]
        app.logger.debug(f"Setting value of {joint.name}: {math.radians(yaw_value)}, {math.radians(pitch_value)}, {math.radians(roll_value)}")
        joint.angles[1] = joint.dof[1] * math.radians(yaw_value)
        joint.angles[0] = joint.dof[0] * math.radians(pitch_value)
        joint.angles[2] = joint.dof[2] * math.radians(roll_value)

    return plot_all(), selected_joint_label


if __name__ == '__main__':
    app.run_server(debug=True)
