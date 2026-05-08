//! Parse a canonical Onshape document URL into its identifiers.

use anyhow::{anyhow, Result};
use regex::Regex;

#[derive(Debug, Clone)]
pub struct OnshapeRef {
    pub did: String,
    /// One of `w` (workspace), `v` (version), `m` (microversion).
    pub wvm: char,
    pub wvmid: String,
    pub eid: String,
}

/// Parse a URL of the form
/// `https://cad.onshape.com/documents/{did}/(w|v|m)/{wvmid}/e/{eid}`.
///
/// Extra path / query segments after the element id are tolerated.
pub fn parse(url: &str) -> Result<OnshapeRef> {
    let re = Regex::new(
        r"documents/([0-9a-fA-F]{24})/([wvm])/([0-9a-fA-F]{24})/e/([0-9a-fA-F]{24})",
    )
    .expect("static regex");
    let caps = re
        .captures(url)
        .ok_or_else(|| anyhow!("URL does not look like an Onshape document: {url}"))?;
    Ok(OnshapeRef {
        did: caps[1].to_lowercase(),
        wvm: caps[2].chars().next().unwrap(),
        wvmid: caps[3].to_lowercase(),
        eid: caps[4].to_lowercase(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_workspace_url() {
        let r = parse("https://cad.onshape.com/documents/0123456789abcdef01234567/w/0123456789abcdef01234568/e/0123456789abcdef01234569").unwrap();
        assert_eq!(r.did, "0123456789abcdef01234567");
        assert_eq!(r.wvm, 'w');
        assert_eq!(r.eid, "0123456789abcdef01234569");
    }

    #[test]
    fn parses_version_url_with_trailing() {
        let r = parse("https://cad.onshape.com/documents/aaaaaaaaaaaaaaaaaaaaaaaa/v/bbbbbbbbbbbbbbbbbbbbbbbb/e/cccccccccccccccccccccccc?x=1").unwrap();
        assert_eq!(r.wvm, 'v');
        assert_eq!(r.wvmid, "bbbbbbbbbbbbbbbbbbbbbbbb");
    }

    #[test]
    fn rejects_bogus() {
        assert!(parse("https://example.com/").is_err());
    }
}
