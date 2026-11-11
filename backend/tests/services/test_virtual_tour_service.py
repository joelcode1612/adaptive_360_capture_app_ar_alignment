import io
import uuid
from datetime import datetime, timezone

from PIL import Image

from sitesense.db.models.virtual_tour import CaptureStatus, VirtualTour, VirtualTourCapture
from sitesense.domain.entities import CaptureMetadata
from sitesense.services.capture_storage import LocalCaptureStorage
from sitesense.services.virtual_tour_service import VirtualTourService


class FakeTourRepository:
    def __init__(self) -> None:
        self.tours: dict[uuid.UUID, VirtualTour] = {}

    async def create(self, *, property_id, owner_id, expected_capture_count):
        tour = VirtualTour(
            id=uuid.uuid4(),
            property_id=property_id,
            owner_id=owner_id,
            expected_capture_count=expected_capture_count,
            status=CaptureStatus.uploading,
            captures=[],
        )
        self.tours[tour.id] = tour
        return tour

    async def get_for_owner(self, tour_id, owner_id):
        tour = self.tours.get(tour_id)
        return tour if tour is not None and tour.owner_id == owner_id else None

    async def get_capture(self, tour_id, target_index):
        tour = self.tours[tour_id]
        return next((c for c in tour.captures if c.target_index == target_index), None)

    async def add_capture(self, capture):
        self.tours[capture.tour_id].captures.append(capture)
        return capture

    async def mark_stitch_pending(self, tour, manifest_key):
        tour.status = CaptureStatus.stitch_pending
        tour.stitch_manifest_key = manifest_key
        return tour


def jpeg_bytes(width: int = 640, height: int = 480) -> bytes:
    out = io.BytesIO()
    Image.new("RGB", (width, height), "white").save(out, format="JPEG")
    return out.getvalue()


def test_image_validation_accepts_portrait_minimum_resolution():
    extension, width, height = VirtualTourService._validate_image(
        jpeg_bytes(480, 640), "image/jpeg"
    )

    assert extension == "jpg"
    assert (width, height) == (480, 640)


async def test_capture_retry_is_idempotent_and_finalise_writes_manifest(tmp_path):
    repository = FakeTourRepository()
    service = VirtualTourService(
        repository,
        LocalCaptureStorage(tmp_path),
        max_capture_bytes=1_000_000,
        max_capture_count=30,
    )
    owner_id = uuid.uuid4()
    tour = await service.create_upload_session(
        property_id=uuid.uuid4(), owner_id=owner_id, expected_capture_count=1
    )
    metadata = CaptureMetadata(
        target_index=1,
        captured_at=datetime.now(timezone.utc),
        pose={"quaternion_w": 1.0, "station_drift_m": 0.02},
    )

    first, duplicate = await service.upload_capture(
        tour_id=tour.id,
        owner_id=owner_id,
        metadata=metadata,
        image=jpeg_bytes(),
        content_type="image/jpeg",
    )
    repeated, duplicate_retry = await service.upload_capture(
        tour_id=tour.id,
        owner_id=owner_id,
        metadata=metadata,
        image=jpeg_bytes(),
        content_type="image/jpeg",
    )

    assert duplicate is False
    assert duplicate_retry is True
    assert repeated.id == first.id
    assert len(tour.captures) == 1

    finalised = await service.finalise_for_stitching(tour_id=tour.id, owner_id=owner_id)
    assert finalised.status is CaptureStatus.stitch_pending
    assert finalised.stitch_manifest_key is not None
    assert (tmp_path / finalised.stitch_manifest_key).is_file()
