use core::ops;


/// Normalize an angle in [-π, π[
pub const fn normalize_radians_pi_pi(mut x: f32) -> f32 {
    while x >= core::f32::consts::PI {
        x -= core::f32::consts::TAU;
    }
    while x < -core::f32::consts::PI {
        x += core::f32::consts::TAU;
    }
    x
}


pub type Matrix33 = [f32; 9];

pub const MATRIX33_IDENTITY: [f32; 9] = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
];

/// Multiply two 3x3 matrices
pub fn mult_matrix33(a: &Matrix33, b: &Matrix33) -> Matrix33 {
    let mut result = [0.0; 9];
    for j in 0..3 {
        for i in 0..3 {
            let mut s = 0_f32;
            for k in 0..3 {
                s += a[i + k * 3] * b[k + j * 3];
            }
            result[i + j * 3] = s;
        }
    }
    result
}

/// Multiply a vector with 3x3 matrix (`V × M`)
pub fn mult_matrix33_vec(m: &Matrix33, v: &[f32; 3]) -> [f32; 3] {
    let mut result: [f32; 3] = [0.0; 3];
    for i in 0..3 {
        for k in 0..3 {
            result[k] += m[i + k * 3] * v[i];
        }
    }
    result
}


/// 2D vector for linear coordinates
#[derive(Default, Clone, Copy)]
pub struct XY {
    pub x: f32,
    pub y: f32,
}

impl XY {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Return the vector length
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Return a unit vectory of same direction, or (0, 0)
    pub fn unit(&self) -> Self {
        let d = self.length();
        if d == 0.0 {
            XY::new(0.0, 0.0)
        } else {
            self / d
        }

    }

    /// Add angle information
    pub const fn with_a(&self, a: f32) -> XYA {
        XYA { x: self.x, y: self.y, a }
    }
}


/// 3D vector for linear and angle coordinates
#[derive(Default, Clone, Copy)]
pub struct XYA {
    pub x: f32,
    pub y: f32,
    pub a: f32,
}

impl XYA {
    pub const fn new(x: f32, y: f32, a: f32) -> Self {
        Self { x, y, a }
    }

    pub const fn xy(&self) -> XY {
        XY { x: self.x, y: self.y }
    }
}


impl ops::Add for &XY {
    type Output = XY;

    fn add(self, other: Self) -> XY {
        XY {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl ops::Sub for &XY {
    type Output = XY;

    fn sub(self, other: Self) -> XY {
        XY {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl ops::Neg for &XY {
    type Output = XY;

    fn neg(self) -> XY {
        XY {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl ops::Mul<f32> for &XY {
    type Output = XY;

    fn mul(self, k: f32) -> XY {
        XY {
            x: self.x * k,
            y: self.y * k,
        }
    }
}

impl ops::Div<f32> for &XY {
    type Output = XY;

    fn div(self, k: f32) -> XY {
        XY {
            x: self.x / k,
            y: self.y / k,
        }
    }
}

impl ops::AddAssign<&XY> for XY {
    fn add_assign(&mut self, other: &XY) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl ops::SubAssign<&XY> for XY {
    fn sub_assign(&mut self, other: &XY) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl ops::MulAssign<f32> for XY {
    fn mul_assign(&mut self, k: f32) {
        self.x *= k;
        self.y *= k;
    }
}

impl ops::DivAssign<f32> for XY {
    fn div_assign(&mut self, k: f32) {
        self.x /= k;
        self.y /= k;
    }
}

impl ops::Add<XY> for XY { type Output = XY; fn add(self, other: Self) -> XY { &self + &other } }
impl ops::Sub<XY> for XY { type Output = XY; fn sub(self, other: Self) -> XY { &self - &other } }
impl ops::Neg for XY { type Output = XY; fn neg(self) -> XY { -&self } }
impl ops::Mul<f32> for XY { type Output = XY; fn mul(self, k: f32) -> XY { &self * k } }
impl ops::Div<f32> for XY { type Output = XY; fn div(self, k: f32) -> XY { &self / k } }
impl ops::AddAssign<XY> for XY { fn add_assign(&mut self, other: XY) { *self += &other; } }
impl ops::SubAssign<XY> for XY { fn sub_assign(&mut self, other: XY) { *self -= &other; } }

impl ops::Mul<&XY> for f32 { type Output = XY; fn mul(self, v: &XY) -> XY { v * self } }
impl ops::Mul<XY> for f32 { type Output = XY; fn mul(self, v: XY) -> XY { v * self } }


/// Generic structure to store (x,y,a) triplets
pub struct PackXYA<T> {
    pub x: T,
    pub y: T,
    pub a: T,
}

