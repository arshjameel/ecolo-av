use super::*;

fn fields(datatype: Datatype, size: u32) -> Vec<Field<'static>> {
    ["x", "y", "z"]
        .into_iter()
        .zip(0..)
        .map(|(name, i)| Field::new(name, size * i, datatype))
        .collect()
}

fn all_xyz(cloud: &CloudView<'_>) -> Result<Vec<[f64; 3]>, CloudError> {
    Ok(xyz(cloud)?.collect())
}

#[test]
fn xyz_comes_out_of_an_organised_cloud_with_padding() {
    // Two rows (beams) of one point: x, y, z float32 then a ring, 16 bytes a point;
    // each row padded to 24 bytes.
    let mut data = Vec::new();
    for p in [[1.0f32, 2.0, 3.0], [f32::INFINITY, f32::NAN, 0.5]] {
        p.iter().for_each(|v| data.extend(v.to_le_bytes()));
        data.extend([0u8; 12]);
    }
    let cloud = CloudView {
        fields: fields(Datatype::Float32, 4),
        width: 1,
        height: 2,
        point_step: 16,
        row_step: 24,
        is_bigendian: false,
        data: &data,
    };
    let points = all_xyz(&cloud).expect("a good cloud");
    assert_eq!(points[0], [1.0, 2.0, 3.0]);
    assert!(points[1][0].is_infinite() && points[1][1].is_nan());
    // The last row needs only its point, not its padding: 24 + 16 bytes.
    let exact = CloudView {
        data: &data[..40],
        ..cloud.clone()
    };
    assert_eq!(all_xyz(&exact).map(|p| p.len()), Ok(2));
    let short = CloudView {
        data: &data[..39],
        ..cloud.clone()
    };
    assert_eq!(
        all_xyz(&short),
        Err(CloudError::TooShort {
            points: 2,
            bytes: 39
        })
    );
    let no_z = CloudView {
        fields: cloud.fields[..2].to_vec(),
        ..cloud
    };
    assert_eq!(all_xyz(&no_z), Err(CloudError::MissingField("z".into())));
}

#[test]
fn doubles_and_big_endian_are_read_too() {
    let data: Vec<u8> = [1.5f64, -2.0, 0.25]
        .iter()
        .flat_map(|v| v.to_be_bytes())
        .collect();
    let cloud = CloudView {
        fields: fields(Datatype::Float64, 8),
        width: 1,
        height: 1,
        point_step: 24,
        row_step: 24,
        is_bigendian: true,
        data: &data,
    };
    assert_eq!(all_xyz(&cloud), Ok(vec![[1.5, -2.0, 0.25]]));
    let ints = CloudView {
        fields: fields(Datatype::Int32, 4),
        ..cloud
    };
    assert_eq!(all_xyz(&ints), Err(CloudError::WrongType("x".into())));
}

#[test]
fn a_field_outside_the_point_or_of_several_values_is_refused() {
    let mut cloud = CloudView {
        fields: fields(Datatype::Float32, 4),
        width: 0,
        height: 1,
        point_step: 12,
        row_step: 0,
        is_bigendian: false,
        data: &[],
    };
    cloud.fields[2].offset = 9;
    assert_eq!(all_xyz(&cloud), Err(CloudError::OutsidePoint("z".into())));
    cloud.fields[2].offset = 8;
    cloud.fields[1].count = 2;
    assert_eq!(all_xyz(&cloud), Err(CloudError::WrongType("y".into())));
    cloud.fields[1].count = 1;
    assert_eq!(all_xyz(&cloud), Ok(Vec::new()));
}

#[test]
fn every_number_type_reads_as_f64() {
    // One point: i8, u8, i16, u16, i32, u32, f32, f64, little-endian.
    let mut data = Vec::new();
    data.extend((-3i8).to_le_bytes());
    data.extend(200u8.to_le_bytes());
    data.extend((-300i16).to_le_bytes());
    data.extend(60_000u16.to_le_bytes());
    data.extend((-70_000i32).to_le_bytes());
    data.extend(4_000_000_000u32.to_le_bytes());
    data.extend(0.1f32.to_le_bytes());
    data.extend(0.1f64.to_le_bytes());
    let names = ["a", "b", "c", "d", "e", "f", "g", "h"];
    let mut offset = 0;
    let fields = names
        .iter()
        .zip(Datatype::ALL)
        .map(|(&name, d)| {
            let f = Field::new(name, offset, d);
            offset += d.size() as u32;
            f
        })
        .collect();
    let cloud = CloudView {
        fields,
        width: 1,
        height: 1,
        point_step: offset,
        row_step: offset,
        is_bigendian: false,
        data: &data,
    };
    let rows = read_columns(&cloud, &names).expect("every type");
    assert_eq!(
        rows,
        vec![vec![
            -3.0,
            200.0,
            -300.0,
            60_000.0,
            -70_000.0,
            4e9,
            f64::from(0.1f32),
            0.1
        ]]
    );
    assert_eq!(
        read_columns(&cloud, &["a", "nope"]),
        Err(CloudError::MissingField("nope".into()))
    );
}

#[test]
fn an_optional_field_may_be_missing_but_not_wrong() {
    let cloud = CloudView {
        fields: fields(Datatype::Float32, 4),
        width: 0,
        height: 0,
        point_step: 12,
        row_step: 0,
        is_bigendian: false,
        data: &[],
    };
    let floats = [Datatype::Float32];
    assert_eq!(cloud.optional_column("intensity", &floats), Ok(None));
    assert!(cloud
        .optional_column("x", &floats)
        .is_ok_and(|c| c.is_some()));
    assert_eq!(
        cloud.optional_column("x", &[Datatype::Uint32]),
        Err(CloudError::WrongType("x".into()))
    );
    assert!(cloud.is_empty());
}

#[test]
fn a_column_reads_nan_from_bytes_too_short() {
    let cloud = CloudView {
        fields: fields(Datatype::Float32, 4),
        width: 0,
        height: 0,
        point_step: 12,
        row_step: 0,
        is_bigendian: false,
        data: &[],
    };
    let z = cloud.column("z", &Datatype::FLOATS).expect("z");
    assert_eq!(z.datatype(), Datatype::Float32);
    assert!(z.read(&[0; 4]).is_nan());
}

#[test]
fn written_points_read_back() {
    let points = [[1.0, -2.5, 0.125], [10.25, 0.0, 3.0]];
    let data = write_xyz32(points);
    assert_eq!(data.len(), 2 * XYZ32_STEP as usize);
    let cloud = CloudView {
        fields: XYZ32_FIELDS.to_vec(),
        width: 2,
        height: 1,
        point_step: XYZ32_STEP,
        row_step: 2 * XYZ32_STEP,
        is_bigendian: false,
        data: &data,
    };
    assert_eq!(all_xyz(&cloud), Ok(points.to_vec()));
}

#[test]
fn datatype_numbers_are_pointfields() {
    assert_eq!(Datatype::from_code(7), Some(Datatype::Float32));
    assert_eq!(Datatype::from_code(6), Some(Datatype::Uint32));
    assert_eq!(Datatype::from_code(0), None);
    assert_eq!(Datatype::from_code(9), None);
    assert!(Datatype::ALL
        .iter()
        .all(|d| Datatype::from_code(d.code()) == Some(*d)));
}
