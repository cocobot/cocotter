//! Procedural textures + PNG loader.
//!
//! Playmat SVGs are rasterized out-of-band (e.g. with Inkscape or resvg);
//! the runtime only consumes the resulting PNG for fast startup.

use std::path::Path;

use bevy::asset::RenderAssetUsages;
use bevy::image::Image;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

/// Generate a tileable wood-grain texture modulated on `base` (sRGB, 0..1).
///
/// The grain runs along the texture's X axis, which is fine for wall
/// cuboids where the longest dimension dominates the UV mapping.
pub fn wood_grain(base: [f32; 3], width: u32, height: u32) -> Image {
    let mut bytes = Vec::with_capacity((width * height * 4) as usize);
    for y in 0..height {
        for x in 0..width {
            let fy = y as f32;
            let fx = x as f32;

            // Main grain: long horizontal stripes from a sum of sines on y.
            let stripes = (fy * 0.12).sin() * 0.10
                + (fy * 0.27 + fx * 0.003).sin() * 0.05
                + (fy * 0.63).sin() * 0.03;

            // Warp along x so stripes meander slightly.
            let warp = (fx * 0.02 + fy * 0.07).sin() * 0.02;

            // Occasional darker knots / cracks.
            let knot_seed = (fx * 0.017 + fy * 0.013).sin()
                * (fx * 0.029 + fy * 0.011).cos();
            let knot = if knot_seed > 0.82 { -0.12 } else { 0.0 };

            // Brightness modulation, centered on 1.0.
            let factor = (1.0 + stripes + warp + knot).clamp(0.65, 1.15);

            let r = (base[0] * factor * 255.0).clamp(0.0, 255.0) as u8;
            let g = (base[1] * factor * 255.0).clamp(0.0, 255.0) as u8;
            let b = (base[2] * factor * 255.0).clamp(0.0, 255.0) as u8;
            bytes.extend_from_slice(&[r, g, b, 255]);
        }
    }
    rgba_to_image(bytes, width, height)
}

fn rgba_to_image(rgba: Vec<u8>, w: u32, h: u32) -> Image {
    Image::new(
        Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        TextureDimension::D2,
        rgba,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

/// Load a PNG file as a Bevy `Image`. Supports 8-bit grayscale, RGB and RGBA.
pub fn load_png(path: &Path) -> Result<Image, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("opening {:?}: {e}", path))?;
    let decoder = png::Decoder::new(file);
    let mut reader = decoder.read_info().map_err(|e| format!("png header: {e}"))?;
    let info = reader.info().clone();
    let w = info.width;
    let h = info.height;
    let mut buf = vec![0u8; reader.output_buffer_size()];
    reader.next_frame(&mut buf).map_err(|e| format!("png decode: {e}"))?;
    let rgba = match info.color_type {
        png::ColorType::Rgba => buf,
        png::ColorType::Rgb => {
            let mut out = Vec::with_capacity(buf.len() / 3 * 4);
            for p in buf.chunks_exact(3) {
                out.extend_from_slice(&[p[0], p[1], p[2], 255]);
            }
            out
        }
        png::ColorType::GrayscaleAlpha => {
            let mut out = Vec::with_capacity(buf.len() * 2);
            for p in buf.chunks_exact(2) {
                out.extend_from_slice(&[p[0], p[0], p[0], p[1]]);
            }
            out
        }
        png::ColorType::Grayscale => {
            let mut out = Vec::with_capacity(buf.len() * 4);
            for &g in &buf {
                out.extend_from_slice(&[g, g, g, 255]);
            }
            out
        }
        other => return Err(format!("unsupported PNG color type: {:?}", other)),
    };
    Ok(rgba_to_image(rgba, w, h))
}
