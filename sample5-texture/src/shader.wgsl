struct CameraUniform {
    view: mat4x4<f32>,
    projection: mat4x4<f32>,
};

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var texture_diffuse: texture_2d<f32>;

@group(1) @binding(1)
var sampler_diffuse: sampler;

@vertex
fn vs_main(
    vs_in: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;

    out.clip_position = camera.projection * camera.view * vec4<f32>(vs_in.position, 1.0);
    out.uv = vs_in.uv;

    return out;
}

@fragment
fn fs_main(fs_in: VertexOutput) -> @location(0) vec4<f32> {
    var color = textureSample(texture_diffuse, sampler_diffuse, fs_in.uv);

    return color;
}
